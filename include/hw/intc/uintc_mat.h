#ifndef HW_INTC_UINTC_MAT_H
#define HW_INTC_UINTC_MAT_H

#include "hw/sysbus.h"

#define TYPE_UINTC_MAT "riscv.uintc.mat"

#define UINTC_MAT(obj) OBJECT_CHECK(UINTCMatState, (obj), TYPE_UINTC_MAT)

typedef struct UINTCMatState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;
    uint32_t num_senders;
    uint32_t num_receivers;
    uint32_t hartid_base;
    uint32_t num_contexts;

    QemuMutex mutex;

    /*
     * Per (sender, receiver) pair, arranged per receiver.
     * Each sender takes up num_words_per_sender words in both arrays.
     */
    uint32_t num_words_per_sender;
    uint32_t num_words_per_receiver;
    uint32_t *pending;
    uint32_t *enable;

    /* Per sender */
    uint32_t *sender_uiid;
    bool *status;

    /* Per receiver */
    uint32_t *receiver_uiid;

    /* Per context */
    uint32_t *listen;
    uint32_t *active_irq_count;
    qemu_irq *user_soft_irqs;
} UINTCMatState;

DeviceState *uintc_mat_create(hwaddr addr, uint32_t num_senders,
                              uint32_t num_receivers, uint32_t hartid_base,
                              uint32_t num_contexts);

enum {
    UINTC_MAT_MAX_SENDERS = 4096,
    UINTC_MAT_MAX_RECEIVERS = 4096,
    UINTC_MAT_MAX_CONTEXTS = 2048,
    UINTC_MAT_SIZE = 0x4000000,
    UINTC_MAT_SENDER_BASE = 0,
    UINTC_MAT_RECEIVER_BASE = 0x2000000,
    UINTC_MAT_CONTEXT_BASE = 0x0,
};

#endif
