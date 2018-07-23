
#include <nan.h>

#include "bsec_integration.h"
#include "bsec_serialized_configurations_iaq.h"

#ifndef IO_ORIGIN_BME680_SRC_BME680_WRAP_H_
#define IO_ORIGIN_BME680_SRC_BME680_WRAP_H_

namespace io_origin_bme680 {

class Bme680 : public Nan::ObjectWrap {
 public:
  static NAN_MODULE_INIT(Init);

 private:
  explicit Bme680(unsigned int i2c_channel);
  ~Bme680();

  static NAN_METHOD(New);
  static NAN_METHOD(Start);

  static Nan::Persistent<v8::Function> constructor;
};

}  // namespace io_origin_bme680

#endif  // IO_ORIGIN_BME680_SRC_BME680_WRAP_H_
