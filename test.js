var Bme680 = require("./");
var obj;
var sensor = new Bme680(1);
function print() {
  Console.log("yasss");
}
//sensor.start(print);
sensor.start( function (values) {
  console.log("here in js" + values);
});
