#include "max1166x.h"

void MAX1166X_init(MAX1166X_Handle *handle, MAX1166X_Type type, device_write_pin_ptr write_cs, device_spi_write_ptr write_cmd, device_write_pin_ptr write_chsel) {
    handle->type=type;
    handle->write_cs=write_cs;
    handle->write_cmd=write_cmd;
    handle->write_chsel=write_chsel;
    handle->current_channel=MAX1166X_CHANNEL1;
    
    switch (type) {
        case MAX11661:
            handle->bits=8;
            handle->channels=1;
            break;
        case MAX11662:
            handle->bits=8;
            handle->channels=2;
            break;
        case MAX11663:
            handle->bits=10;
            handle->channels=1;
            break;
        case MAX11664:
            handle->bits=10;
            handle->channels=2;
            break;
        case MAX11665:
            handle->bits=12;
            handle->channels=1;
            break;
        case MAX11666:
            handle->bits=12;
            handle->channels=2;
            break;
    }
    
}

void MAX1166X_read_next_channel(MAX1166X_Handle *handle, uint16_t *val) {
    uint8_t data[2];
    data[0]=0;
    data[1]=0;
    
    handle->write_cs(true);
    if (handle->channels>1) {
        handle->write_cmd(handle, data, 1);
        handle->current_channel=1-handle->current_channel;
        handle->write_chsel(handle->current_channel);
        handle->write_cmd(handle, &(data[1]), 1);
    } else {
        handle->write_cmd(handle, data, 2);
    }
    
    
    handle->write_cs(false);
    
    uint16_t d=data[0];
    switch (handle->bits) {
        case 8:
            *val = (d << 1) | (data[1] >> 7);
            break;
        case 10:
            *val = (d << 3) | (data[1] >> 5);
            break;
        case 12:
            *val = (d << 5) | (data[1] >> 3);
            break;
    }
    
}

void MAX1166X_power_down(MAX1166X_Handle *handle) {
    // pull cs high between 2nd and 10th clock
    
    uint8_t data[1];
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 1);
    handle->write_cs(false);
}

void MAX1166X_wake_up(MAX1166X_Handle *handle) {
    // pull cs high after 10th clock
    
    uint8_t data[2];
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 2);
    handle->write_cs(false);
}