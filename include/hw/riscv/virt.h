/*
 * QEMU RISC-V VirtIO machine interface
 *
 * Copyright (c) 2017 SiFive, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_RISCV_VIRT_H
#define HW_RISCV_VIRT_H

#include "hw/riscv/riscv_hart.h"
#include "hw/sysbus.h"
#include "hw/block/flash.h"

#define TYPE_RISCV_VIRT_MACHINE MACHINE_TYPE_NAME("virt")
#define RISCV_VIRT_MACHINE(obj) \
    OBJECT_CHECK(RISCVVirtState, (obj), TYPE_RISCV_VIRT_MACHINE)

typedef struct {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
    PFlashCFI01 *flash[2];

    void *fdt;
    int fdt_size;
} RISCVVirtState;

enum {
    VIRT_DEBUG,
    VIRT_MROM,
    VIRT_TEST,
    VIRT_RTC,
    VIRT_CLINT,
    VIRT_PLIC,
    VIRT_UART0,
    VIRT_UART1,
    VIRT_UART2,
    VIRT_UART3,
    VIRT_UART4,
    VIRT_VIRTIO,
    VIRT_FLASH,
    VIRT_DRAM,
    VIRT_PCIE_MMIO,
    VIRT_PCIE_PIO,
    VIRT_PCIE_ECAM
};

enum {
    UART0_IRQ = 10,
    RTC_IRQ = 11,
    UART1_IRQ = 12,
    UART2_IRQ = 13,
    UART3_IRQ = 14,
    UART4_IRQ = 15,
    VIRTIO_IRQ = 1, /* 1 to 8 */
    VIRTIO_COUNT = 8,
    PCIE_IRQ = 0x20, /* 32 to 35 */
    VIRTIO_NDEV = 0x35 /* Arbitrary maximum number of interrupts */
};

#define VIRT_PLIC_HART_CONFIG "MSU,MSU,MSU,MSU"
#define VIRT_PLIC_NUM_SOURCES 127
#define VIRT_PLIC_NUM_PRIORITIES 7
#define VIRT_PLIC_PRIORITY_BASE 0x04
#define VIRT_PLIC_PENDING_BASE 0x1000
#define VIRT_PLIC_ENABLE_BASE 0x2000
#define VIRT_PLIC_ENABLE_STRIDE 0x80
#define VIRT_PLIC_CONTEXT_BASE 0x200000
#define VIRT_PLIC_CONTEXT_STRIDE 0x1000

#define FDT_PCI_ADDR_CELLS    3
#define FDT_PCI_INT_CELLS     1
#define FDT_PLIC_ADDR_CELLS   0
#define FDT_PLIC_INT_CELLS    1
#define FDT_INT_MAP_WIDTH     (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + 1 + \
                               FDT_PLIC_ADDR_CELLS + FDT_PLIC_INT_CELLS)

#if defined(TARGET_RISCV32)
#define VIRT_CPU TYPE_RISCV_CPU_BASE32
#elif defined(TARGET_RISCV64)
#define VIRT_CPU TYPE_RISCV_CPU_BASE64
#endif

#endif
