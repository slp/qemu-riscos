/*
 * Raspberry Pi emulation (c) 2012 Gregory Estrade
 * This code is licensed under the GNU GPLv2 and later.
 */

#include "hw/sysbus.h"
#include <sys/types.h>         
#include <sys/socket.h>
#include <string.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#define TYPE_BCM2835_TODO "bcm2835_todo"
#define BCM2835_TODO(obj) \
        OBJECT_CHECK(bcm2835_todo_state, (obj), TYPE_BCM2835_TODO)

#define GPIO_MEM_SIZE 0xA0
#define TERM_BIT_PIN 25
#define TERM_READ_PIN 7
#define TERM_WRITE_PIN 8
#define TOS_BIT_PIN 14
#define TOS_READ_PIN 18
#define TOS_WRITE_PIN 15

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    int gpio7_terminal_read;
    int gpio15_tos_write;
    int gpio14_tos_bit;
    int bit_index;
    int str_index;
    unsigned int bits[8];
    char str[256];
} bcm2835_todo_state;

/* Get pin number */
static int get_pin(uint64_t value) {
    int pin = 0;
    while ((pin < 32) && ((value & 1) == 0)) {
        value = value >> 1;
        pin += 1;
    }
    
    if(pin >=32) {
        fprintf(stderr, "[QEMU][Raspi] Unknown pin number %d!", pin);
        exit(1);
    }
    return pin;
}

static int setup_socket(void) {
    const int SERVER_PORT = 8989;
    struct sockaddr_in serv_addr;
    struct hostent *local;
    int socketfd;

    local = gethostbyname("localhost");
    if (local == NULL) {
        fprintf(stderr, "[QEMU][Raspi] Can not get hostname");
        exit(1);
    }
    socketfd = 0;
    socketfd = socket(AF_INET, SOCK_STREAM, 0);

    if (socketfd < 0) {
        fprintf(stderr, "[QEMU][Raspi] ERROR opening socket");
        exit(1);
    }

    memset((char *)&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)local->h_addr, (char *)&serv_addr.sin_addr.s_addr, local->h_length);
    serv_addr.sin_port = htons(SERVER_PORT);

    if (connect(socketfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0){
        fprintf(stderr, "[QEMU][Raspi] ERROR connecting(%d): %s", errno, strerror(errno));
        exit(1);
    }
    return socketfd;
}

/* Send string through socet */
static void send_string(char *str, int len) {
    int n;
    int socket_fd;
    char buf[256];
    
    socket_fd = setup_socket();
    
    n = write(socket_fd, str, len);

    if (n < 0) {
        fprintf(stderr, "[qemu][raspi] error writing to socket\n");
        exit(1);
    }    

    /* Read response */
    memset(buf, 0, sizeof(buf));
    n = read(socket_fd, buf, 255);

    if (n < 0) {
        fprintf(stderr, "[QEMU][Raspi] TODO WRITE ERROR %s reading from socket %d\n", strerror(errno), n);
        exit(1);
    }    

    if (strcmp(buf, "OK") != 0){
        fprintf(stderr, "[QEMU][Raspi] ERROR response from socket\n");
        exit(1);
    }

    fprintf(stderr,"[QEMU] TOS GPIO: write %s to GPIO server\n", str);
    close(socket_fd);
}


static uint64_t read_gpio(bcm2835_todo_state *s, unsigned int offset) {
    unsigned int value = 0;
    if (offset != 0x34) {
        fprintf(stderr, "[QEMU][TOS] Warnning! GPIO read from unknown offset %x!", offset);
        return (uint64_t)0xffffffff;
    }

    value = value | ((s->gpio7_terminal_read) << TERM_READ_PIN);
    value = value | ((s->gpio14_tos_bit) << TOS_BIT_PIN);
    value = value | ((s->gpio15_tos_write)  << TOS_WRITE_PIN);
    return (uint64_t) value; 
}

static void write_char(bcm2835_todo_state *s) {
    char v = 0;
    int i = 0, j = 7;
    
    if (s->str_index > 255) {
        fprintf(stderr, "[QEMU][TOS] Reach string buffer limit!"); 
        return;
    }

    for (i=0,j=7; i<8; i++,j--) {
        v += (s->bits[i] << j);
    }
    
    s->str[s->str_index] = v;
    s->str_index += 1;
    
    if(v == 0) {
        // If meet '\0', send string and reset string buffer
        send_string(s->str, s->str_index);
        memset(s->str, 0, sizeof(s->str));
        s->str_index = 0;
    }
    return;
}

static void write_gpio(bcm2835_todo_state *s, uint64_t value, short bit) {
    int pin; 
    
    pin = get_pin(value);
    switch (pin) {
        
            case TOS_BIT_PIN:
            if ((s->gpio7_terminal_read == -1) || (s->gpio15_tos_write == -1)) {
                // Initialize haven't done yet.
                return;
            }

            if (s->bit_index >= 8) {
                fprintf(stderr, "[QEMU][TOS] Warning! Bit buffer full!\n");
                return; 
            }
            s->bits[s->bit_index] = bit;
            s->bit_index += 1;
            break;
        
        case TERM_READ_PIN:
            /* TERM_READ_PIN should be read only */
            break;

        case TOS_WRITE_PIN:
            if (s->gpio15_tos_write == -1) {
                s->gpio15_tos_write = bit;
                fprintf(stderr, "[QEMU][TOS] Initialize: PIN 15 TOS_WRITE = %d\n", (int)bit);
                return; 
            }
            s->gpio15_tos_write = bit;
            if(bit == 1) {
                if (s->bit_index >= 8) {
                    write_char(s);
                    memset(s->bits, 0, sizeof(s->bits));
                    s->bit_index = 0;
                }
                s->gpio7_terminal_read = 1;
            } else {
                s->gpio7_terminal_read = 0;
            }
            break;
        case 18:
            break;
        default:
            fprintf(stderr, "[QEMU][Raspi] Warnning! write to unknown pin %d\n", pin);
    }     
}


/*
 * Read from socket 
 */
static uint64_t bcm2835_todo_read(void *opaque, hwaddr offset,
    unsigned size)
{
    int n;
    unsigned int value;
    const char *cmd = NULL;
    char buf[256];
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
 
    int socketfd = 0;

    switch (offset) {
        case 0x0:
        case 0x4:
        case 0x8:
        case 0xC:
        case 0x10:
        case 0x14:
            return 0;

        case 0x34:
        case 0x38:
            return read_gpio(s, (unsigned int)offset);
        case 0x94:
            return 0;

        default:
            cmd = "";
            fprintf(stderr, "[QEMU][Raspi] Warning Read from unknown offset %x!\n", (unsigned int)offset);
            return 0xffffffff;
    }
    
    /* Send msg to server */
    bzero(buf, 256);
    sprintf(buf, "%s#R", cmd);

    socketfd = setup_socket();
    n = write(socketfd, buf, strlen(buf));

    if (n < 0) {
        fprintf(stderr, "[QEMU][Raspi] ERROR %s writing to socket\n", strerror(errno));
        exit(1);
    }    

    /* Read response */
    bzero(buf, 256);
    n = read(socketfd, buf, 255);

    if (n < 0) {
        fprintf(stderr, "[QEMU][Raspi] READ ERROR %s reading from socket %s\n", strerror(errno), buf);
        exit(1);
    }    

    value = atoi(buf);
    close(socketfd);
    fprintf(stderr, "[QEMU] TOS GPIO: read %d from %x\n", value, (unsigned int)offset);
    return (uint64_t)value;
}

/*
 * Write to socket 
 * Format: 
 *  Write to offest Cmd#Pin
 *  Response should be "OK"
 */
static void bcm2835_todo_write(void *opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
    
    switch (offset) {
        case 0x0:
        case 0x4:
        case 0x8:
        case 0xC:
        case 0x10:
        case 0x14:
            return ;

        case 0x1C:
            write_gpio(s, value, 1);
            break;
        case 0x20:        
            fprintf(stderr, "[QEMU][Raspi] GPIO write 1 to PIN set 1!\n");
            return;
        case 0x28:
            write_gpio(s, value, 0);
            break;
        case 0x2C:
            fprintf(stderr, "[QEMU][Raspi] GPIO write 0 to PIN set 1!\n");
            break;
        case 0x94:
            break;
        case 0x98:
            break;
        case 0x9C:
            break;
        default:
            fprintf(stderr, "[QEMU][Raspi] Warning Write to unknown offset %x!\n", (unsigned int)offset);
    }
    return;
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
    //const int SERVER_PORT = 8989;

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

    /* initialize gpio pins */
    s->gpio7_terminal_read = 0;
    s->gpio14_tos_bit = 0;
    s->gpio15_tos_write = -1;
    s->bit_index = 0;
    s->str_index = 0;
    memset(s->bits, 0, sizeof(s->bits));
    memset(s->str, 0, sizeof(s->str));
    
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
