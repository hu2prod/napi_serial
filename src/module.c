#include <node_api.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include "runtime_native.c"
#include "array_size_t.c"
#include "hash.c"

////////////////////////////////////////////////////////////////////////////////////////////////////
//    context
////////////////////////////////////////////////////////////////////////////////////////////////////

u32 free_context_counter = 0;
void* free_context_fifo = NULL;
void** alloc_context_hash = NULL;

void free_capture_context_fifo_ensure_init() {
  if (free_context_fifo == NULL) {
    free_context_fifo = array_size_t_alloc(16);
    alloc_context_hash = hash_size_t_alloc();
  }
}

struct Serial_context {
  // nodejs aware of this
  char* dev_path;
  u32 ctx_idx;
  u32 baud_rate;
  int termios_baud_rate;
  int fd;
};

struct Serial_context* context_by_id(int id) {
  return (struct Serial_context*)hash_size_t_get(alloc_context_hash, id);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//    util
////////////////////////////////////////////////////////////////////////////////////////////////////
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

int set_interface_attribs(int fd, int speed, int parity) {
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    printf("Error from tcgetattr: %s\n", strerror(errno));
    return -1;
  }
  
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;   // disable break processing
  tty.c_lflag = 0;    // no signaling chars, no echo,
          // no canonical processing
  tty.c_oflag = 0;    // no remapping, no delays
  tty.c_cc[VMIN]  = 0;      // read doesn't block
  tty.c_cc[VTIME] = 5;      // 0.5 seconds read timeout
  
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  
  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
          // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return -1;
  }
  if (tcsetattr(fd, TCSAFLUSH, &tty) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return -1;
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

napi_value serial_open(napi_env env, napi_callback_info info) {
  napi_status status;
  
  napi_value ret_dummy;
  status = napi_create_int32(env, 0, &ret_dummy);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_dummy");
    return ret_dummy;
  }
  
  size_t argc = 3;
  napi_value argv[3];
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
    return ret_dummy;
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  size_t dev_path_len;
  status = napi_get_value_string_utf8(env, argv[0], NULL, 0, &dev_path_len);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Can't get dev_path_len");
    return ret_dummy;
  }
  
  size_t tmp;
  char *dev_path = malloc(dev_path_len);
  status = napi_get_value_string_utf8(env, argv[0], dev_path, dev_path_len+1, &tmp);
  
  i32 baud_rate;
  status = napi_get_value_int32(env, argv[1], &baud_rate);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid i32 was passed as argument of baud_rate");
    return ret_dummy;
  }
  
  i32 open_flag;
  status = napi_get_value_int32(env, argv[1], &open_flag);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid i32 was passed as argument of open_flag");
    return ret_dummy;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // https://linux.die.net/man/3/termios
  int termios_baud_rate;
  switch(baud_rate) {
    case 50    : termios_baud_rate = B50    ; break;
    case 75    : termios_baud_rate = B75    ; break;
    case 110   : termios_baud_rate = B110   ; break;
    case 134   : termios_baud_rate = B134   ; break;
    case 150   : termios_baud_rate = B150   ; break;
    case 200   : termios_baud_rate = B200   ; break;
    case 300   : termios_baud_rate = B300   ; break;
    case 600   : termios_baud_rate = B600   ; break;
    case 1200  : termios_baud_rate = B1200  ; break;
    case 1800  : termios_baud_rate = B1800  ; break;
    case 2400  : termios_baud_rate = B2400  ; break;
    case 4800  : termios_baud_rate = B4800  ; break;
    case 9600  : termios_baud_rate = B9600  ; break;
    case 19200 : termios_baud_rate = B19200 ; break;
    case 38400 : termios_baud_rate = B38400 ; break;
    case 57600 : termios_baud_rate = B57600 ; break;
    case 115200: termios_baud_rate = B115200; break;
    case 230400: termios_baud_rate = B230400; break;
    default:
      napi_throw_error(env, NULL, "bad baud rate");
      return ret_dummy;
  }
  
  // TODO flags
  int fd = open(dev_path, O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    napi_throw_error(env, NULL, "failed to open");
    return ret_dummy;
  }
  if (0 != set_interface_attribs(fd, termios_baud_rate, 0)) {
    napi_throw_error(env, NULL, "set_interface_attribs failed");
    return ret_dummy;
  }
  tcflush(fd, TCIFLUSH);
  tcflush(fd, TCOFLUSH);
  
  struct Serial_context* ctx;
  int ctx_idx;
  if (array_size_t_length_get(free_context_fifo)) {
    ctx = (struct Serial_context*)array_size_t_pop(free_context_fifo);
    ctx_idx = ctx->ctx_idx;
  } else {
    // alloc
    ctx = (struct Serial_context*)malloc(sizeof(struct Serial_context));
    ctx_idx = free_context_counter++;
    ctx->ctx_idx = ctx_idx;
    alloc_context_hash = hash_size_t_set(alloc_context_hash, ctx_idx, (size_t)ctx);
  }
  
  ctx->dev_path = dev_path;
  ctx->baud_rate = baud_rate;
  ctx->termios_baud_rate = termios_baud_rate;
  ctx->fd = fd;
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  napi_value ret_idx;
  status = napi_create_int32(env, ctx->ctx_idx, &ret_idx);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_idx");
    return ret_dummy;
  }
  
  return ret_idx;
}

napi_value serial_close(napi_env env, napi_callback_info info) {
  napi_status status;
  
  napi_value ret_dummy;
  status = napi_create_int32(env, 0, &ret_dummy);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_dummy");
    return ret_dummy;
  }
  
  size_t argc = 1;
  napi_value argv[1];
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
    return ret_dummy;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  i32 ctx_idx;
  status = napi_get_value_int32(env, argv[0], &ctx_idx);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid i32 was passed as argument of ctx_idx");
    return ret_dummy;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //    checks
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  struct Serial_context* ctx = context_by_id(ctx_idx);
  if (!ctx) {
    napi_throw_error(env, NULL, "Invalid ctx_idx");
    return ret_dummy;
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  close(ctx->fd);
  free(ctx->dev_path);
  ctx->dev_path = NULL;
  free_context_fifo = array_size_t_push(free_context_fifo, (size_t)ctx);
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  return ret_dummy;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//    RW
////////////////////////////////////////////////////////////////////////////////////////////////////

napi_value serial_read_buf(napi_env env, napi_callback_info info) {
  napi_status status;
  
  napi_value ret_dummy;
  status = napi_create_int32(env, 0, &ret_dummy);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_dummy");
    return ret_dummy;
  }
  
  size_t argc = 3;
  napi_value argv[3];
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
    return ret_dummy;
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  i32 ctx_idx;
  status = napi_get_value_int32(env, argv[0], &ctx_idx);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid i32 was passed as argument of ctx_idx");
    return ret_dummy;
  }
  
  
  u8 *data_dst;
  size_t data_dst_len;
  status = napi_get_buffer_info(env, argv[1], (void**)&data_dst, &data_dst_len);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid buffer was passed as argument of data_dst");
    return ret_dummy;
  }
  
  
  i32 timeout_mcs;
  status = napi_get_value_int32(env, argv[2], &timeout_mcs);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid i32 was passed as argument of timeout_mcs");
    return ret_dummy;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  struct Serial_context* ctx = context_by_id(ctx_idx);
  if (!ctx) {
    napi_throw_error(env, NULL, "Invalid ctx_idx");
    return ret_dummy;
  }
  
  i32 total_rx_size = 0;
  int fd = ctx->fd;
  
  const int wait_resolution_mcs = 50;
  
  timeout_mcs /= wait_resolution_mcs;
  
  for(int retry=0;retry<=timeout_mcs;retry++) {
    if (data_dst_len <= 0) break; // can't write anything
    fd_set fds;
    struct timeval tv;
    
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    tv.tv_sec = 0;
    tv.tv_usec = wait_resolution_mcs;
    
    int select_res = select(fd + 1, &fds, NULL, NULL, &tv);
    if (select_res < 0) {
      printf("Error from select: %s\n", strerror(errno));
      napi_throw_error(env, NULL, "select error");
      return ret_dummy;
    }
    if (select_res == 0) {
      // timeout
      continue;
    }
    
    int real_rx_size = read(fd, data_dst, data_dst_len);
    total_rx_size += real_rx_size;
    data_dst      += real_rx_size;
    data_dst_len  -= real_rx_size;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  napi_value ret_size;
  status = napi_create_int32(env, total_rx_size, &ret_size);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_size");
    return ret_dummy;
  }
  
  return ret_size;
}

napi_value serial_write_buf(napi_env env, napi_callback_info info) {
  napi_status status;
  
  napi_value ret_dummy;
  status = napi_create_int32(env, 0, &ret_dummy);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_dummy");
    return ret_dummy;
  }
  
  size_t argc = 2;
  napi_value argv[2];
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
    return ret_dummy;
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  i32 ctx_idx;
  status = napi_get_value_int32(env, argv[0], &ctx_idx);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid i32 was passed as argument of ctx_idx");
    return ret_dummy;
  }
  
  
  u8 *data_src;
  size_t data_src_len;
  status = napi_get_buffer_info(env, argv[1], (void**)&data_src, &data_src_len);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid buffer was passed as argument of data_src");
    return ret_dummy;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  struct Serial_context* ctx = context_by_id(ctx_idx);
  if (!ctx) {
    napi_throw_error(env, NULL, "Invalid ctx_idx");
    return ret_dummy;
  }
  
  int write_size = write(ctx->fd, data_src, data_src_len);
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  napi_value ret_size;
  status = napi_create_int32(env, write_size, &ret_size);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_size");
    return ret_dummy;
  }
  
  return ret_size;
}

napi_value serial_write_str(napi_env env, napi_callback_info info) {
  napi_status status;
  
  napi_value ret_dummy;
  status = napi_create_int32(env, 0, &ret_dummy);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_dummy");
    return ret_dummy;
  }
  
  size_t argc = 2;
  napi_value argv[2];
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
    return ret_dummy;
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  i32 ctx_idx;
  status = napi_get_value_int32(env, argv[0], &ctx_idx);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid i32 was passed as argument of ctx_idx");
    return ret_dummy;
  }
  
  
  size_t data_src_len;
  status = napi_get_value_string_utf8(env, argv[1], NULL, 0, &data_src_len);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Can't get data_src_len");
    return ret_dummy;
  }
  
  size_t tmp;
  char *data_src = malloc(data_src_len);
  status = napi_get_value_string_utf8(env, argv[1], data_src, data_src_len+1, &tmp);
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  struct Serial_context* ctx = context_by_id(ctx_idx);
  if (!ctx) {
    napi_throw_error(env, NULL, "Invalid ctx_idx");
    return ret_dummy;
  }
  
  int write_size = write(ctx->fd, data_src, data_src_len);
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  napi_value ret_size;
  status = napi_create_int32(env, write_size, &ret_size);
  
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value ret_size");
    return ret_dummy;
  }
  
  return ret_size;
}
////////////////////////////////////////////////////////////////////////////////////////////////////


napi_value Init(napi_env env, napi_value exports) {
  napi_status status;
  napi_value fn;
  
  __alloc_init();
  free_capture_context_fifo_ensure_init();
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  status = napi_create_function(env, NULL, 0, serial_open, NULL, &fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to wrap native function");
  }
  
  status = napi_set_named_property(env, exports, "open", fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to populate exports");
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  status = napi_create_function(env, NULL, 0, serial_close, NULL, &fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to wrap native function");
  }
  
  status = napi_set_named_property(env, exports, "close", fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to populate exports");
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  status = napi_create_function(env, NULL, 0, serial_read_buf, NULL, &fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to wrap native function");
  }
  
  status = napi_set_named_property(env, exports, "read_buf", fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to populate exports");
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  status = napi_create_function(env, NULL, 0, serial_write_buf, NULL, &fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to wrap native function");
  }
  
  status = napi_set_named_property(env, exports, "write_buf", fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to populate exports");
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  status = napi_create_function(env, NULL, 0, serial_write_str, NULL, &fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to wrap native function");
  }
  
  status = napi_set_named_property(env, exports, "write_str", fn);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to populate exports");
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  
  return exports;
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)