
#include "bme680_wrap.h"

#include <stdio.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <sstream>
#include <string.h>

namespace io_origin_bme680 {

namespace {
int i2c_fd_;

void init_i2c (int i2c_channel) {
  std::ostringstream i2cFileNameStream;
  i2cFileNameStream << "/dev/i2c-" << i2c_channel;
  std::string i2cFileName = i2cFileNameStream.str();
  if ((i2c_fd_ = open(i2cFileName.c_str(), O_RDWR)) < 0)
    throw std::runtime_error("failed to open bus.");
   
  if (ioctl(i2c_fd_, I2C_SLAVE, BME680_I2C_ADDR_PRIMARY) < 0) 
    throw std::runtime_error("failed to device.");
}

void sleep_ms(uint32_t t_ms) {
  sleep(t_ms/1000);
}

typedef struct {
  struct timeval startTimeVal;
} TIMER_usecCtx_t;

void TIMER_usecStart(TIMER_usecCtx_t* ctx) {
  gettimeofday(&ctx->startTimeVal, NULL);
}
unsigned int TIMER_usecElapsedUs(TIMER_usecCtx_t* ctx) {
  unsigned int rv;
  struct timeval nowTimeVal;
  gettimeofday(&nowTimeVal, NULL);

  rv = 1000000 * (nowTimeVal.tv_sec - ctx->startTimeVal.tv_sec) + nowTimeVal.tv_usec - ctx->startTimeVal.tv_usec;
  return rv;
}

TIMER_usecCtx_t timer;

int64_t get_timestamp_us() { 
  // handle timer overflow 
  int64_t system_current_time = 0;
  system_current_time = (int64_t) TIMER_usecElapsedUs(&timer);
  return system_current_time;
}

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
   float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status) {
 // printf("TS: % "PRIi64" , T: %.2f degC, P: %.2f hPa, H %.2f %%rH, IAQ %.2f (% "PRIi8")\n ", timestamp, temperature,
   //   pressure / 100.0f, humidity, iaq, iaq_accuracy);
}

int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len) {
  int rslt = 0;
  uint8_t reg[16];
  reg[0] = reg_addr;
  for(int i = 1; i < data_len +1; i++)
    reg[i] = reg_data_ptr[i-1];

  if (write(i2c_fd_, reg, data_len+1) != data_len+1) 
    throw std::runtime_error("write opration failed.");
  
  return rslt;
}

int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len) {
  int rslt = 0;
  uint8_t reg[1];
  reg[0]=reg_addr;
  if (write(i2c_fd_, reg, 1) != 1) 
    throw std::runtime_error("write opration failed."); 
 
   if (read(i2c_fd_, reg_data_ptr, data_len) != data_len) 
     throw std::runtime_error("read opration failed.");
 
  return rslt;
}

uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer) {
  return 0;
}

uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer) {
  memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq)/sizeof(bsec_config_iaq[0]));
  n_buffer = sizeof(bsec_config_iaq)/sizeof(bsec_config_iaq[0]);

  return n_buffer;  
}

void state_save(const uint8_t *state_buffer, uint32_t length) {
}

class StartWorker : public Nan::AsyncWorker { 
  public:
    StartWorker(Nan::Callback *callback)
        : Nan::AsyncWorker(callback) {}
    ~StartWorker() {}
    
    void Execute() {
        bsec_iot_loop( sleep_ms, get_timestamp_us, output_ready, state_save, 10000 );        
    }

    void HandleOKCallback () {
    /* Local<String> temp = Nan::New<String>("Temp").ToLocalChecked();
    Local<v8::Object> results = Nan::New<Object>();
    Nan::Set(obj, temp, Nan::New<Number>(temperature));
    Local<v8::Value> argv [] = { results };
    callback->Call(1, argv); */
    } 
};

} // namespace

Nan::Persistent<v8::Function> Bme680::constructor;

Bme680::Bme680(unsigned int i2c_channel) {
  //return_values_init ret;
  TIMER_usecStart(&timer);
  init_i2c(i2c_channel); 
  bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, sleep_ms, state_load, config_load);
}
Bme680::~Bme680() {}

NAN_MODULE_INIT(Bme680::Init) {
  Nan::HandleScope scope;

  // Prepare constructor template
  v8::Local<v8::FunctionTemplate> tpl = Nan::New<v8::FunctionTemplate>(New);
  tpl->SetClassName(Nan::New("Bme680").ToLocalChecked());
  tpl->InstanceTemplate()->SetInternalFieldCount(1);

  // Prototype
  Nan::SetPrototypeMethod(tpl, "start", Start);

  constructor.Reset(tpl->GetFunction());
  target->Set(Nan::New("Bme680").ToLocalChecked(), tpl->GetFunction());
}

NAN_METHOD(Bme680::New) {
  if (info.Length() != 1) {
    Nan::ThrowTypeError("Unexpected number of constructor arguments, one argument expected.");
    return;
  }
  if (!info[0]->IsNumber() || info[0]->NumberValue() < 0 || info[0]->NumberValue() > 16) {
    Nan::ThrowTypeError("Unexpected constructor arguments, expected valid i2c channel.");
    return;
  }
  if (info.IsConstructCall()) {
    // Invoked as constructor: `new BME680(...)`
    unsigned int i2c_channel = info[0]->IntegerValue();
    try {
      Bme680* wrapper = new Bme680(i2c_channel);
      wrapper->Wrap(info.This());
    } catch (const std::exception& e) {
      Nan::ThrowError(e.what());
      return;
    }  
    info.GetReturnValue().Set(info.This());
  } else {
    // Invoked as plain function `BME680(...)`, turn into construct call.
    const int argc = 1;
    v8::Local<v8::Value> argv[argc] = { info[0] };
    v8::Local<v8::Function> cons = Nan::New<v8::Function>(constructor);
    Nan::MaybeLocal<v8::Object> maybe_instance = Nan::NewInstance(cons, argc, argv);
    if (maybe_instance.IsEmpty()) {
      Nan::ThrowError("Could not create new Bme680 instance.");
      return;
    }
    info.GetReturnValue().Set(maybe_instance.ToLocalChecked());
  }
}

NAN_METHOD(Bme680::Start) {
  if (info.Length() != 1) {
    Nan::ThrowTypeError("Unexpected number of arguments for setValue(), expected 1 argument.");
    return;
  }
  Nan::Callback *callback = new Nan::Callback(info[0].As<v8::Function>());
  Bme680* wrapper = ObjectWrap::Unwrap<Bme680>(info.Holder());
  Nan::AsyncQueueWorker(new StartWorker(callback));
}

}  // namespace io_origin_temperature
