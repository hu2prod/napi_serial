# napi_serial
## use

    napi_serial = require("napi_serial")
    
    # zero alloc inside NAPI for RW
    read_buffer = Buffer.alloc 1024
    ctx = napi_serial.open "/dev/ttyACM0", 115200, 0
    write_len = napi_serial.write_str ctx, "ping"
    read_len  = napi_serial.read_buf  ctx, read_buffer, 50*1000 # last parameter is timeout_mcs for waiting of result read
    read_buffer_slice = read_buffer.slice 0, read_len
    console.log "read_buffer_slice", read_buffer_slice
    
    write_len = napi_serial.write_buf ctx, Buffer.from "ping"
    
    napi_serial.close ctx
    
## Motivation and notes
I'm tired with serialport module \
e.g. https://github.com/serialport/node-serialport/issues/529

This module doesn't support async and open flags yet, but it's fully usable for me \
Also this module is much more simplier that serialport (except for hash, array implementation) \
Contexts are borrowed from napi_v4l2
