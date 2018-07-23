//Copyright Message

#include <nan.h>

#include "bme680_wrap.h"

namespace io_origin_bme680 {

void InitAll(v8::Local<v8::Object> exports) {
  Bme680::Init(exports);
}

NODE_MODULE(bme680, InitAll)

}  // namespace io_origin_temperature
