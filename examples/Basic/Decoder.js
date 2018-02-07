/**
 * Use as payload decoder function.
 *
 * 12 8E 00 21 09 5A (port 2)
 * 
 * {
 *   "event": "interval",
 *   "battery": 4750,
 *   "light": 33,
 *   "temperature": 23.94
 * }
 */

function Decoder(bytes, port) {
  var decoded = {};

  var events = {
    1: 'setup',
    2: 'interval',
    3: 'motion',
    4: 'button'
  };

  decoded.event = events[port];
  decoded.battery = (bytes[0] << 8) + bytes[1];
  decoded.light = (bytes[2] << 8) + bytes[3];
  if (bytes[4] & 0x80)
    decoded.temperature = ((0xffff << 16) + (bytes[4] << 8) + bytes[5]) / 100;
  else
    decoded.temperature = ((bytes[4] << 8) + bytes[5]) / 100;

  return decoded;
}
