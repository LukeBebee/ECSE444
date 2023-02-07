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
Called with the address of the kalman_state object (integer - R0) and the current measurement (single-percision float - S0)
Return int 0 if everything works, -1 if there is error
FPSCR bits [0,3] cleared at start of function to check for undesired arithmetic conditions
*/
kalman:
	// Push and pop non-scratch registers to prevent clobbering (I think that's the term)
	VPUSH {S1-S6} // S6 will be used for computation, the rest will be to store variables from kalman_state structure

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
	MOV R2, #0b1111	// R2 has bits to clear (overflow, underflow, division by zero, invalid operation)
	BIC R1, R1, R2	// Clear bits indicated by R2
	VMSR FPSCR, R1	// Write to FPSCR with cleared bits


	// Start Kalman algorithm: *****************************************
	// p = p + q -- potential overflow
	VADD.F32 S4, S4, S1
	// k = p/(p + r)-- potential overflow and div by 0
	VADD.F32 S5, S4, S2
	VDIV.F32 S5, S4, S5
	// x = x + k*(measurement - x) -- potential overflow and underflow
	VSUB.F32 S6, S0, S3
	VMLA.F32 S3, S5, S6
	// p = (1 - k) * p -- potential overflow and underflow
	VMOV.F32 S6, #1.0
	VSUB.F32 S6, S6, S5
	VMUL.F32 S4, S4, S6
	// End Kalman algorithm ********************************************

	MOV R3, #-1 // let R4 store output for now, assume error

	// Check for overflow, underflow, division by 0
	VMRS R1, FPSCR	// R1 has FPSCR contents
	//MOV R2, #0b1111	// R2 has bits we are checking (this is done on line 38 so no need execute)
	AND R1, R1, R2	// R1 has the 4 bits that were possibly triggered by the arithmetic operations
	CMP R1, #0
	BNE no_store	// If 0, no flags so we continue and store. If not zero, skip over store

	// Update structure values (store register values in memory)
	VSTM.F32 R0, {S1-S5}
	MOV R3, #0	// only here if no errors, so return value should be 0
	no_store:

	MOV R0, R3 	// set output, will be -1 here if we skipped store (in case of error) and 0 if no error

	// Restore non-scratch registers then leave function
	VPOP {S1-S6}
	BX LR
