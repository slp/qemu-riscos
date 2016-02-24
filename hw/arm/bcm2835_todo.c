/*
 * Raspberry Pi emulation (c) 2012 Gregory Estrade
 * This code is licensed under the GNU GPLv2 and later.
 */

#include "hw/sysbus.h"
#include "sys/shm.h"
#include "sys/ipc.h"

#define TYPE_BCM2835_TODO "bcm2835_todo"
#define BCM2835_TODO(obj) \
        OBJECT_CHECK(bcm2835_todo_state, (obj), TYPE_BCM2835_TODO)

#define GPIO_MEM_SIZE 0xA0

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    int* gpio_shm;
    
} bcm2835_todo_state;

static uint64_t bcm2835_todo_read(void *opaque, hwaddr offset,
    unsigned size)
{
    fprintf(stderr, "[QEMU] TOS GPIO: read");
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
    
    fprintf(stderr, "[QEMU] TOS GPIO: read %x from 202000%x size:(%d)\n", (int)*(s->gpio_shm+(int)offset/4), (int)offset, (int)size);

    return (unsigned)*(s->gpio_shm+(int)offset/4);
}

static void bcm2835_todo_write(void *opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    fprintf(stderr,"[QEMU] TOS GPIO: write %x to 202000%x \n", (int)value, (int)offset);
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
    *(s->gpio_shm+(int)offset/4) = (unsigned)value;
    if (((int)offset == 0x1c) || ((int)offset == 0x20)) {
        // write to 0x34 or 0x38
        *(s->gpio_shm+(int)offset/4+6) |= (unsigned)value;
        fprintf(stderr,"[QEMU] TOS GPIO: set %x to %x \n", (int)offset+0x18, *(s->gpio_shm+(int)offset/4+6)) ;
    } else if (((int)offset == 0x28) || ((int)offset == 0x2c)) {
        // write to 0x34 or 0x38
        *(s->gpio_shm+(int)offset/4+3) &= ~(unsigned)value;
        fprintf(stderr,"[QEMU] TOS GPIO: clear %x to %x \n", (int)offset+0x0c, *(s->gpio_shm+(int)offset/4+6)) ;
    } 
}

static const MemoryRegionOps bcm2835_todo_ops = {
    .read = bcm2835_todo_read,
    .write = bcm2835_todo_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_bcm2835_todo = {
    .name = TYPE_BCM2835_TODO,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static int bcm2835_todo_init(SysBusDevice *sbd)
{
    int shmid;

    DeviceState *dev = DEVICE(sbd);
    bcm2835_todo_state *s = BCM2835_TODO(dev);

    /* Modified by Yeqing Yan 
     * Change size to 0xAO */
    /* memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_todo_ops, s,
        TYPE_BCM2835_TODO, 0x1000000);*/
    memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_todo_ops, s,
        TYPE_BCM2835_TODO, 0xa0);
    sysbus_init_mmio(sbd, &s->iomem);

    vmstate_register(dev, -1, &vmstate_bcm2835_todo, s);

    key_t key = 6666;

    if ((shmid = shmget(key, GPIO_MEM_SIZE, IPC_CREAT | 0666)) < 0 ) {
        fprintf(stderr, "[QEMU] TOS GPIO failed to create share memory)\n");
    }

    s->gpio_shm = shmat(shmid, NULL, 0);
    if (s->gpio_shm == (int *) -1) {
        fprintf(stderr, "[QEMU] TOS Attach share memory failed\n");
    }
    
    int i;
    /* Reset shared memory */
    for (i = 0; i < 40; i ++) {
        *(s->gpio_shm+i) = 0; 
    }   

    return 0;
}

static void bcm2835_todo_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

    sdc->init = bcm2835_todo_init;
}

static TypeInfo bcm2835_todo_info = {
    .name          = TYPE_BCM2835_TODO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(bcm2835_todo_state),
    .class_init    = bcm2835_todo_class_init,
};

static void bcm2835_todo_register_types(void)
{
    type_register_static(&bcm2835_todo_info);
}

type_init(bcm2835_todo_register_types)
