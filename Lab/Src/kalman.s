/*
 * kalman.s
 *
 */

.global kalman

/**
This assembly file contains a single function (kalman)

*/



/**
Kalman function updates values of a Kalman object based on it's current state and a current measurement
Called with the address of the Kalman object (integer - R0) and the current measurement (single-percision float - S0)
*/
kalman:
	// Push and pop non-scratch registers to prevent clobbering (I think that's the term)
	VPUSH {S1-S6} // Can prevent use of S6 by storing r after calculation and using that, but cannot then use store multiple

	// Load single precision floating point registers with variables from structure
	VLDM.F32 R0, {S1-S5}
	/*
	Assume at this point variables are in the following registers
	measurement: S0 current measurement
	q: S1 process noise variance
	r: S2 measurement noise variance
	x: S3 value
	p: S4 estimation error covariance
	k: S5 kalman gain
	*/

	// Clear FPSCR exception bits that we will be checking later
	VMRS R1, FPSCR	// R1 has FPSCR contents
	MOV R2, 0b1111	// R2 has bits to clear
	EOR R1, R1, R2	// Clear bits indicated by R2
	VMSR R1, FPSCR	// Clear FPSCR bits


	// Start Kalman algorithm: *****************************************
	// p = p + q -- potential overflow
	VADD.F32 S4, S4, S1
	// k = p/(p + r)-- potential overflow and div by 0
	VADD.F32 S5, S4, S2
	VDIV.F32 S5, S4, S5
	// x = x + k*(measurement - x) -- potential overflow and underflow
	VMLA.F32 S3, S5, S6
	// p = (1 - k) * p -- potential overflow and underflow
	VMOV.F32 S6, #1.0
	VSUB.F32 S6, S6, S5
	VMUL.F32 S4, S4, S6
	// End Kalman algorithm ********************************************


	// Check for overflow, underflow, division by 0
	VMRS R1, FPSCR	// R1 has FPSCR contents
	//MOV R2, #0b1111	// R2 has bits we are checking (this is done on line 39 so no need to actually execute)
	ORR R1, R1, R2	// R1 has the 4 bits that were possibly triggered by the arithmetic operations
	CMP R1, #0
	BNE no_store	// If 0, no flags so we continue and store. If not zero, skip over store

	// Update structure values (store register values in memory)
	VSTM.F32 R0, {S1-S5}
	no_store:

	// Restore non-scratch registers then leave function
	VPOP {S1-S6}
	BX LR
