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

function decodeUplink(input) {
	return {
		data: {
			event: {
				1: "setup",
				2: "interval",
				3: "motion",
				4: "button"
			} [input.fPort],
			battery: (input.bytes[0] << 8) + input.bytes[1],
			light: (input.bytes[2] << 8) + input.bytes[3],
			temperature: (((input.bytes[4] & 0x80) ? (0xffff << 16) : 0) + (input.bytes[4] << 8) + input.bytes[5]) / 100
		},
		warnings: [],
		errors: []
	};
}
