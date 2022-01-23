#include "qemu/osdep.h"

#include "hw/intc/uintc_mat.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "target/riscv/cpu.h"

#define UINTC_MAT_DEBUG false

static void uintc_mat_print_state(UINTCMatState *uintc)
{
    uint32_t i, j;
    uint32_t *ptr;

    qemu_log("senders:   ");
    for (i = uintc->num_senders - 1; i != -1; i--) {
        qemu_log("%u%c ", uintc->sender_uiid[i], "XO"[uintc->status[i]]);
    }
    qemu_log("\n");

    qemu_log("receivers: ");
    for (i = 0; i < uintc->num_receivers; i++) {
        qemu_log("%u ", uintc->receiver_uiid[i]);
    }
    qemu_log("\n");

    qemu_log("contexts:  ");
    for (i = 0; i < uintc->num_contexts; i++) {
        qemu_log("%u(%u) ", uintc->listen[i], uintc->active_irq_count[i]);
    }
    qemu_log("\n");

    qemu_log("enable:\n");
    ptr = uintc->enable;
    for (i = 0; i < uintc->num_receivers; i++) {
        qemu_log("receiver%4u: ", i);
        for (j = uintc->num_words_per_receiver - 1; j != -1; j--) {
            qemu_log("%08x", ptr[j]);
        }
        qemu_log("\n");
        ptr += uintc->num_words_per_receiver;
    }

    qemu_log("pending:\n");
    ptr = uintc->pending;
    for (i = 0; i < uintc->num_receivers; i++) {
        qemu_log("receiver%4u: ", i);
        for (j = uintc->num_words_per_receiver - 1; j != -1; j--) {
            qemu_log("%08x", ptr[j]);
        }
        qemu_log("\n");
        ptr += uintc->num_words_per_receiver;
    }

    qemu_log("\n");
}

static bool within(hwaddr addr, hwaddr start, uint64_t size)
{
    return start <= addr && addr < start + size;
}

static void uintc_mat_update(UINTCMatState *uintc)
{
    uint32_t i;

    for (i = 0; i < uintc->num_contexts; i++) {
        bool level = uintc->active_irq_count[uintc->listen[i]] > 0;
        if (level) {
            qemu_irq_raise(uintc->user_soft_irqs[uintc->hartid_base + i]);
        }
        /*
         * We currently do not lower irqs in qemu. Otherwise interrupts
         * delivered by (supervisor) software might be erased.
         * This may differ from real hardware behavior.
         */
    }

    if (UINTC_MAT_DEBUG) {
        uintc_mat_print_state(uintc);
    }
}

static uint64_t uintc_mat_withdraw_read(UINTCMatState *uintc,
                                        uint32_t sender_id)
{
    uint32_t i;

    /* Word and bit offset in each row */
    uint32_t word_offset = sender_id >> 5;
    uint32_t bit = (uint32_t)1 << (sender_id & 0x1F);

    /* Pointer to the word in one row */
    uint32_t *pending = uintc->pending + word_offset;

    for (i = 1; i < uintc->num_receivers; i++) {
        if (*pending & bit) {
            *pending &= ~bit;

            /* Update active interrupt count */
            uint32_t *enable =
                uintc->enable + i * uintc->num_words_per_receiver + word_offset;
            if (*enable & bit) {
                uintc->active_irq_count[i]--;
            }

            return i;
        }
    }

    /* No interrupt found */
    return 0;
}

static uint64_t uintc_mat_per_sender_bitfield_read(UINTCMatState *uintc,
                                                   uint32_t *bitfield,
                                                   uint32_t sender_id,
                                                   uint32_t word_index,
                                                   unsigned size)
{
    uint32_t i;
    uint64_t result = 0;
    uint32_t bit_count = size << 3;

    /* Start of receiver_id to read */
    uint32_t receiver_id_start = word_index << 5;

    /* Word and bit offset in each row */
    uint32_t word_offset = sender_id >> 5;
    uint32_t bit_index = sender_id & 0x1F;

    /* Pointer to the word in one row */
    uint32_t *word = bitfield + word_offset +
                     uintc->num_words_per_receiver * receiver_id_start;

    for (i = 0; i < bit_count; i++) {
        if (receiver_id_start + i >= uintc->num_receivers) {
            break;
        }

        result |= ((*word >> bit_index) & 1) << i;
        word += uintc->num_words_per_sender;
    }

    return result;
}

static uint64_t uintc_mat_claim_read(UINTCMatState *uintc, uint32_t receiver_id)
{
    uint32_t i;
    uint32_t start = uintc->num_words_per_receiver * receiver_id;

    for (i = 0; i < uintc->num_words_per_receiver; i++) {
        uint32_t p = uintc->pending[start + i] & uintc->enable[start + i];
        if (p) {
            /* Find a bit and clear it. */
            int first_bit_index = ctz32(p);
            uint32_t first_bit = (uint32_t)1 << first_bit_index;
            uintc->pending[start + i] &= ~first_bit;

            uintc->active_irq_count[receiver_id]--;

            return (i << 5) | first_bit_index;
        }
    }

    /* No interrupt found */
    return 0;
}

static uint64_t uintc_mat_per_receiver_bitfield_read(UINTCMatState *uintc,
                                                     uint32_t *bitfield,
                                                     uint32_t receiver_id,
                                                     uint32_t word_index,
                                                     unsigned size)
{
    uint32_t *row = bitfield + uintc->num_words_per_receiver * receiver_id;

    uint64_t result = row[word_index];

    /* If size == 8, check if the higher 32 bits are in range */
    if (size == 8 && word_index < uintc->num_words_per_sender - 1) {
        result |= (uint64_t)row[word_index + 1] << 32;
    }

    return result;
}

static void uintc_mat_send_write(UINTCMatState *uintc, uint32_t sender_id,
                                 uint64_t value)
{
    uint32_t i;

    /* Word and bit offset in each row */
    uint32_t word_offset = sender_id >> 5;
    uint32_t bit = (uint32_t)1 << (sender_id & 0x1F);

    for (i = 1; i < uintc->num_receivers; i++) {
        if (uintc->receiver_uiid[i] == value) {
            uint32_t w = i * uintc->num_words_per_receiver + word_offset;

            if (uintc->enable[w] & bit) {
                if (!(uintc->pending[w] & bit)) {
                    uintc->pending[w] |= bit;
                    uintc->active_irq_count[i]++;
                }
                uintc->status[sender_id] = true;
            } else {
                uintc->status[sender_id] = false;
            }

            return;
        }
    }

    uintc->status[sender_id] = false;
}

static void uintc_mat_per_sender_bitfield_write(
    UINTCMatState *uintc, uint32_t *bitfield_to_write, uint32_t *bitfield_other,
    uint32_t sender_id, uint32_t word_index, unsigned size, uint64_t value)
{
    uint32_t i;
    uint32_t bit_count = size << 3;

    /* Start of receiver_id to write */
    uint32_t receiver_id_start = word_index << 5;

    /* Word and bit offset in each row */
    uint32_t word_offset = sender_id >> 5;
    uint32_t bit = 1 << (sender_id & 0x1F);

    /* Pointer to the words in one row */
    uint32_t *word_to_write = bitfield_to_write + word_offset +
                              uintc->num_words_per_receiver * receiver_id_start;
    uint32_t *word_other = bitfield_other + word_offset +
                           uintc->num_words_per_receiver * receiver_id_start;

    for (i = 0; i < bit_count; i++) {
        if (receiver_id_start + i >= uintc->num_receivers) {
            break;
        }

        if (*word_to_write & *word_other & bit) {
            uintc->active_irq_count[receiver_id_start + i]--;
        }

        *word_to_write &= ~bit;
        if ((value >> i) & 1) {
            *word_to_write |= bit;
        }

        if (*word_to_write & *word_other & bit) {
            uintc->active_irq_count[receiver_id_start + i]++;
        }

        word_to_write += uintc->num_words_per_sender;
        word_other += uintc->num_words_per_sender;
    }
}

static void uintc_mat_per_receiver_bitfield_write(
    UINTCMatState *uintc, uint32_t *bitfield_to_write, uint32_t *bitfield_other,
    uint32_t receiver_id, uint32_t word_index, unsigned size, uint64_t value)
{
    uint32_t *row_to_write;
    uint32_t *row_other;

    /* Auxilliary function to write to a 32 bit word. */
    void write_word(
        uint32_t * word_to_write, uint32_t word_other, uint32_t value)
    {
        uintc->active_irq_count[receiver_id] -=
            ctpop32(word_other & *word_to_write);

        *word_to_write = value;

        uintc->active_irq_count[receiver_id] +=
            ctpop32(word_other & *word_to_write);
    }

    row_to_write =
        bitfield_to_write + uintc->num_words_per_receiver * receiver_id;
    row_other = bitfield_other + uintc->num_words_per_receiver * receiver_id;

    write_word(
        &row_to_write[word_index], row_other[word_index], value & 0xFFFFFFFF);

    /* If size == 8, check if the higher 32 bits are in range */
    if (size == 8 && word_index < uintc->num_words_per_sender - 1) {
        write_word(&row_to_write[word_index + 1],
                   row_other[word_index + 1],
                   value >> 32);
    }
}

static uint64_t uintc_mat_read_impl(UINTCMatState *uintc, hwaddr addr,
                                    unsigned size, bool *need_update)
{
    *need_update = false;

    if (UINTC_MAT_DEBUG) {
        qemu_log("reading from %lx\n", addr);
    }

    if ((addr & 0x3) != 0) {
        goto err;
    }

    if (within(addr, UINTC_MAT_CONTEXT_BASE, uintc->num_contexts << 2)) {
        /* listen */
        uint32_t context_id = (addr - UINTC_MAT_CONTEXT_BASE) >> 2;
        return uintc->listen[context_id];
    } else if (within(addr, UINTC_MAT_SENDER_BASE, uintc->num_senders << 13)) {
        uint32_t sender_id = (addr - UINTC_MAT_SENDER_BASE) >> 13;
        hwaddr sender_offset = addr & 0x1FFF;

        if (sender_id == 0) {
            goto err;
        }

        if (sender_offset == 0) {
            /* status */
            return uintc->status[sender_id];
        } else if (sender_offset == 0x1000) {
            /* sender_uiid */
            return uintc->sender_uiid[sender_id];
        } else if (sender_offset == 0x1004) {
            /* withdraw */
            *need_update = true;
            return uintc_mat_withdraw_read(uintc, sender_id);
        } else if (within(sender_offset,
                          0x1800,
                          uintc->num_words_per_sender << 2)) {
            /* enable */

            if (size == 8 && (sender_offset & 0x7) != 0) {
                goto err;
            }

            return uintc_mat_per_sender_bitfield_read(
                uintc,
                uintc->enable,
                sender_id,
                (sender_offset - 0x1800) >> 2,
                size);
        } else if (within(sender_offset,
                          0x1A00,
                          uintc->num_words_per_sender << 2)) {
            /* pending */

            if (size == 8 && (sender_offset & 0x7) != 0) {
                goto err;
            }

            return (uintc_mat_per_sender_bitfield_read(
                uintc,
                uintc->pending,
                sender_id,
                (sender_offset - 0x1A00) >> 2,
                size));
        } else {
            goto err;
        }
    } else if (within(
                   addr, UINTC_MAT_RECEIVER_BASE, uintc->num_receivers << 13)) {
        uint32_t receiver_id = (addr - UINTC_MAT_RECEIVER_BASE) >> 13;
        hwaddr receiver_offset = addr & 0x1FFF;

        if (receiver_id == 0) {
            goto err;
        }

        if (receiver_offset == 0) {
            /* claim */
            *need_update = true;
            return uintc_mat_claim_read(uintc, receiver_id);
        } else if (receiver_offset == 0x1000) {
            /* receiver_uiid */
            return uintc->receiver_uiid[receiver_id];
        } else if (within(receiver_offset,
                          0x1800,
                          uintc->num_words_per_receiver << 2)) {
            /* enable */

            if (size == 8 && (receiver_offset & 0x7) != 0) {
                goto err;
            }

            return uintc_mat_per_receiver_bitfield_read(
                uintc,
                uintc->enable,
                receiver_id,
                ((receiver_offset - 0x1800) >> 2),
                size);
        } else if (within(receiver_offset,
                          0x1A00,
                          uintc->num_words_per_receiver << 2)) {
            /* pending */

            if (size == 8 && (receiver_offset & 0x7) != 0) {
                goto err;
            }

            return uintc_mat_per_receiver_bitfield_read(
                uintc,
                uintc->pending,
                receiver_id,
                ((receiver_offset - 0x1A00) >> 2),
                size);
        } else {
            goto err;
        }
    }

err:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Invalid register read 0x%" HWADDR_PRIx "\n",
                  __func__,
                  addr);
    return 0;
}

static uint64_t uintc_mat_read(void *opaque, hwaddr addr, unsigned size)
{
    UINTCMatState *uintc = opaque;
    uint64_t result;
    bool need_update;

    qemu_mutex_lock(&uintc->mutex);

    result = uintc_mat_read_impl(uintc, addr, size, &need_update);

    if (need_update) {
        uintc_mat_update(uintc);
    }

    qemu_mutex_unlock(&uintc->mutex);

    return result;
}

static void uintc_mat_write_impl(UINTCMatState *uintc, hwaddr addr,
                                 uint64_t value, unsigned size,
                                 bool *need_update)
{
    if (UINTC_MAT_DEBUG) {
        qemu_log("writing to %lx %lu\n", addr, value);
    }

    *need_update = false;

    if ((addr & 0x3) != 0) {
        goto err;
    }

    if (within(addr, UINTC_MAT_CONTEXT_BASE, uintc->num_contexts << 2)) {
        /* listen */
        uint32_t context_id = (addr - UINTC_MAT_CONTEXT_BASE) >> 2;

        *need_update = true;
        uintc->listen[context_id] = value;
    } else if (within(addr, UINTC_MAT_SENDER_BASE, uintc->num_senders << 13)) {
        uint32_t sender_id = (addr - UINTC_MAT_SENDER_BASE) >> 13;
        hwaddr sender_offset = addr & 0x1FFF;

        if (sender_id == 0) {
            goto err;
        }

        if (sender_offset == 0) {
            /* send */

            *need_update = true;
            uintc_mat_send_write(uintc, sender_id, value);
        } else if (sender_offset == 0x1000) {
            /* sender_uiid */
            uintc->sender_uiid[sender_id] = value;
        } else if (within(sender_offset,
                          0x1800,
                          uintc->num_words_per_sender << 2)) {
            /* enable */

            if (size == 8 && (sender_offset & 0x7) != 0) {
                goto err;
            }

            *need_update = true;
            return uintc_mat_per_sender_bitfield_write(
                uintc,
                uintc->enable,
                uintc->pending,
                sender_id,
                (sender_offset - 0x1800) >> 2,
                size,
                value);
        } else if (within(sender_offset,
                          0x1A00,
                          uintc->num_words_per_sender << 2)) {
            /* pending */

            if (size == 8 && (sender_offset & 0x7) != 0) {
                goto err;
            }

            *need_update = true;
            return uintc_mat_per_sender_bitfield_write(
                uintc,
                uintc->pending,
                uintc->enable,
                sender_id,
                (sender_offset - 0x1A00) >> 2,
                size,
                value);
        } else {
            goto err;
        }
    } else if (within(
                   addr, UINTC_MAT_RECEIVER_BASE, uintc->num_receivers << 13)) {
        uint32_t receiver_id = (addr - UINTC_MAT_RECEIVER_BASE) >> 13;
        hwaddr receiver_offset = addr & 0x1FFF;

        if (receiver_id == 0) {
            goto err;
        }

        if (receiver_offset == 0x1000) {
            /* receiver_uiid */
            uintc->receiver_uiid[receiver_id] = value;
        } else if (within(receiver_offset,
                          0x1800,
                          uintc->num_words_per_receiver << 2)) {
            /* enable */

            if (size == 8 && (receiver_offset & 0x7) != 0) {
                goto err;
            }

            *need_update = true;
            return uintc_mat_per_receiver_bitfield_write(
                uintc,
                uintc->enable,
                uintc->pending,
                receiver_id,
                (receiver_offset - 0x1800) >> 2,
                size,
                value);
        } else if (within(receiver_offset,
                          0x1A00,
                          uintc->num_words_per_receiver << 2)) {
            /* pending */

            if (size == 8 && (receiver_offset & 0x7) != 0) {
                goto err;
            }

            *need_update = true;
            return uintc_mat_per_receiver_bitfield_write(
                uintc,
                uintc->pending,
                uintc->enable,
                receiver_id,
                (receiver_offset - 0x1A00) >> 2,
                size,
                value);
        } else {
            goto err;
        }
    }

err:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Invalid register write 0x%" HWADDR_PRIx "\n",
                  __func__,
                  addr);
}

static void uintc_mat_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    UINTCMatState *uintc = opaque;
    bool need_update;

    qemu_mutex_lock(&uintc->mutex);

    uintc_mat_write_impl(uintc, addr, value, size, &need_update);

    if (need_update) {
        uintc_mat_update(uintc);
    }

    qemu_mutex_unlock(&uintc->mutex);
}

static const MemoryRegionOps uintc_mat_ops = {
    .read = uintc_mat_read,
    .write = uintc_mat_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 8 },
};

static void uintc_mat_class_realize(DeviceState *dev, Error **errp)
{
    uint32_t num_words;
    UINTCMatState *uintc = UINTC_MAT(dev);

    /* Initialize memory mapping */
    memory_region_init_io(&uintc->mmio,
                          OBJECT(dev),
                          &uintc_mat_ops,
                          uintc,
                          TYPE_UINTC_MAT,
                          UINTC_MAT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &uintc->mmio);

    qemu_mutex_init(&uintc->mutex);

    /* Allocate storage */
    uintc->num_words_per_sender = (uintc->num_receivers + 31) >> 5;
    uintc->num_words_per_receiver = (uintc->num_senders + 31) >> 5;

    num_words = uintc->num_words_per_receiver * uintc->num_receivers;
    uintc->pending = g_new0(uint32_t, num_words);
    uintc->enable = g_new0(uint32_t, num_words);

    uintc->sender_uiid = g_new0(uint32_t, uintc->num_senders);
    uintc->status = g_new0(bool, uintc->num_senders);

    uintc->receiver_uiid = g_new0(uint32_t, uintc->num_receivers);

    uintc->listen = g_new0(uint32_t, uintc->num_contexts);
    uintc->active_irq_count = g_new0(uint32_t, uintc->num_contexts);

    /*
     * Initialize user software interrupt signals as output gpio of the
     * device
     */
    uintc->user_soft_irqs = g_malloc(sizeof(qemu_irq) * uintc->num_contexts);
    qdev_init_gpio_out(dev, uintc->user_soft_irqs, uintc->num_contexts);

    /* We don't claim mip.USIP since it is writeable by software. */
}

static Property uintc_mat_properties[] = {
    DEFINE_PROP_UINT32("num-senders", UINTCMatState, num_senders, 0),
    DEFINE_PROP_UINT32("num-receivers", UINTCMatState, num_receivers, 0),
    DEFINE_PROP_UINT32("hartid-base", UINTCMatState, hartid_base, 0),
    DEFINE_PROP_UINT32("num-contexts", UINTCMatState, num_contexts, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void uintc_mat_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = uintc_mat_class_realize;
    device_class_set_props(dc, uintc_mat_properties);
}

static const TypeInfo uintc_mat_info = {
    .name = TYPE_UINTC_MAT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(UINTCMatState),
    .class_init = uintc_mat_class_init,
};

DeviceState *uintc_mat_create(hwaddr addr, uint32_t num_senders,
                              uint32_t num_receivers, uint32_t hartid_base,
                              uint32_t num_contexts)
{
    int i;
    DeviceState *dev = qdev_new(TYPE_UINTC_MAT);

    assert(num_senders <= UINTC_MAT_MAX_SENDERS);
    assert(num_receivers <= UINTC_MAT_MAX_RECEIVERS);
    assert(num_contexts <= UINTC_MAT_MAX_CONTEXTS);
    assert(!(addr & 0x7));

    qdev_prop_set_uint32(dev, "num-senders", num_senders);
    qdev_prop_set_uint32(dev, "num-receivers", num_receivers);
    qdev_prop_set_uint32(dev, "hartid-base", hartid_base);
    qdev_prop_set_uint32(dev, "num-contexts", num_contexts);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);

    for (i = 0; i < num_contexts; i++) {
        CPUState *cpu = qemu_get_cpu(hartid_base + i);

        qdev_connect_gpio_out(
            dev, i, qdev_get_gpio_in(DEVICE(cpu), IRQ_U_SOFT));
    }

    return dev;
}

static void uintc_mat_register_types(void)
{
    type_register_static(&uintc_mat_info);
}

type_init(uintc_mat_register_types)
