package main

import (
	"fmt"
	"math"
)

// DebugAngularInstability focuses on angular rate and quaternion issues
func DebugAngularInstability() {
	fmt.Println("🔍 Debugging Angular Rate Instability")
	fmt.Println("=====================================")
	
	engine := NewSimplifiedFlightDynamicsEngine(NewEulerIntegrator())
	
	// Start with proper NED coordinates
	state := NewAircraftState()
	state.Altitude = 3000.0
	state.Position.Z = -3000.0
	state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
	state.Controls.Throttle = 0.7
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("Starting conditions:\n")
	fmt.Printf("   Angular Rate: (%.6f, %.6f, %.6f)\n", 
		state.AngularRate.X, state.AngularRate.Y, state.AngularRate.Z)
	fmt.Printf("   Orientation: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		state.Orientation.W, state.Orientation.X, state.Orientation.Y, state.Orientation.Z)
	
	dt := 0.01
	for step := 0; step < 300; step++ { // Run until we find the issue
		
		// Calculate forces and moments
		components, err := engine.Calculator.CalculateSimplifiedForces(state)
		if err != nil {
			fmt.Printf("❌ Forces error at step %d: %v\n", step, err)
			break
		}
		
		// Check moment magnitudes
		totalMoment := components.TotalMoment.Magnitude()
		if totalMoment > 1e6 { // Very large moments
			fmt.Printf("⚠️  Large total moment at step %d: %.2e N·m\n", step, totalMoment)
			fmt.Printf("   Individual moments: Roll=%.2e, Pitch=%.2e, Yaw=%.2e\n",
				components.Moments.Roll, components.Moments.Pitch, components.Moments.Yaw)
		}
		
		// Calculate state derivatives
		derivatives := engine.Calculator.CalculateStateDerivatives(state, components)
		
		// Check angular acceleration magnitudes
		angAccelMag := derivatives.AngularRateDot.Magnitude()
		if angAccelMag > 100 { // Very large angular accelerations
			fmt.Printf("⚠️  Large angular acceleration at step %d: %.2e rad/s²\n", step, angAccelMag)
			fmt.Printf("   Components: (%.3f, %.3f, %.3f)\n",
				derivatives.AngularRateDot.X, derivatives.AngularRateDot.Y, derivatives.AngularRateDot.Z)
			fmt.Printf("   Current alpha: %.1f° (%.3f rad)\n", state.Alpha*RAD_TO_DEG, state.Alpha)
			fmt.Printf("   Current angular rates: (%.6f, %.6f, %.6f)\n",
				state.AngularRate.X, state.AngularRate.Y, state.AngularRate.Z)
		}
		
		// Check for NaN in derivatives
		if hasNaNInDerivatives(derivatives) {
			fmt.Printf("❌ NaN in derivatives at step %d!\n", step)
			printDerivativesDebug(derivatives, "NaN_DERIVATIVES")
			break
		}
		
		// Perform integration
		newState := engine.Integrator.Integrate(state, derivatives, dt)
		
		// Check for NaN in new state
		if hasNaNValues(newState) {
			fmt.Printf("❌ NaN after integration at step %d!\n", step)
			fmt.Printf("   Previous state was valid\n")
			fmt.Printf("   Derivatives: angular_accel=(%.3f, %.3f, %.3f)\n",
				derivatives.AngularRateDot.X, derivatives.AngularRateDot.Y, derivatives.AngularRateDot.Z)
			
			// Check quaternion integration specifically
			fmt.Printf("\n🔬 Debugging quaternion integration:\n")
			debugQuaternionIntegrationStep(state, derivatives, dt)
			break
		}
		
		// Check quaternion validity
		qMag := math.Sqrt(newState.Orientation.W*newState.Orientation.W + 
						  newState.Orientation.X*newState.Orientation.X + 
						  newState.Orientation.Y*newState.Orientation.Y + 
						  newState.Orientation.Z*newState.Orientation.Z)
		
		if math.Abs(qMag - 1.0) > 0.1 { // Quaternion should be unit length
			fmt.Printf("⚠️  Quaternion denormalization at step %d: magnitude=%.6f\n", step, qMag)
		}
		
		// Progress report every 50 steps
		if step%50 == 0 {
			fmt.Printf("   Step %d: α=%.1f°, q_mag=%.6f, ang_rate_mag=%.6f\n",
				step, newState.Alpha*RAD_TO_DEG, qMag, newState.AngularRate.Magnitude())
		}
		
		state = newState
	}
}

// debugQuaternionIntegrationStep manually steps through quaternion integration
func debugQuaternionIntegrationStep(state *AircraftState, derivatives *StateDerivatives, dt float64) {
	fmt.Printf("   Initial quaternion: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		state.Orientation.W, state.Orientation.X, state.Orientation.Y, state.Orientation.Z)
	fmt.Printf("   Angular rate: (%.6f, %.6f, %.6f)\n",
		state.AngularRate.X, state.AngularRate.Y, state.AngularRate.Z)
	fmt.Printf("   Angular acceleration: (%.6f, %.6f, %.6f)\n",
		derivatives.AngularRateDot.X, derivatives.AngularRateDot.Y, derivatives.AngularRateDot.Z)
	
	// Step 1: Update angular rate
	newAngularRate := state.AngularRate.Add(derivatives.AngularRateDot.Scale(dt))
	fmt.Printf("   New angular rate: (%.6f, %.6f, %.6f)\n",
		newAngularRate.X, newAngularRate.Y, newAngularRate.Z)
	
	// Check for NaN in angular rate
	if math.IsNaN(newAngularRate.X) || math.IsNaN(newAngularRate.Y) || math.IsNaN(newAngularRate.Z) {
		fmt.Printf("   ❌ NaN in new angular rate!\n")
		return
	}
	
	// Step 2: Create omega quaternion
	omegaQuat := Quaternion{W: 0, X: newAngularRate.X, Y: newAngularRate.Y, Z: newAngularRate.Z}
	fmt.Printf("   Omega quaternion: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		omegaQuat.W, omegaQuat.X, omegaQuat.Y, omegaQuat.Z)
	
	// Step 3: Quaternion multiplication for orientation rate
	orientationDot := state.Orientation.Multiply(omegaQuat).Scale(0.5)
	fmt.Printf("   Orientation dot: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		orientationDot.W, orientationDot.X, orientationDot.Y, orientationDot.Z)
	
	// Check for NaN in orientation dot
	if math.IsNaN(orientationDot.W) || math.IsNaN(orientationDot.X) || 
	   math.IsNaN(orientationDot.Y) || math.IsNaN(orientationDot.Z) {
		fmt.Printf("   ❌ NaN in orientation dot after multiplication!\n")
		return
	}
	
	// Step 4: Update orientation
	newOrientation := state.Orientation.Add(orientationDot.Scale(dt))
	fmt.Printf("   Pre-normalize orientation: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		newOrientation.W, newOrientation.X, newOrientation.Y, newOrientation.Z)
	
	// Step 5: Normalize
	finalOrientation := newOrientation.Normalize()
	fmt.Printf("   Final orientation: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		finalOrientation.W, finalOrientation.X, finalOrientation.Y, finalOrientation.Z)
	
	// Check for NaN in final orientation
	if math.IsNaN(finalOrientation.W) || math.IsNaN(finalOrientation.X) || 
	   math.IsNaN(finalOrientation.Y) || math.IsNaN(finalOrientation.Z) {
		fmt.Printf("   ❌ NaN in final orientation!\n")
	} else {
		fmt.Printf("   ✅ Quaternion integration completed successfully\n")
	}
}

// DebugMomentCalculation focuses on moment calculation issues
func DebugMomentCalculation() {
	fmt.Println("\n🔧 Debugging Moment Calculation")
	fmt.Println("===============================")
	
	calc := NewSimplifiedCalculator()
	
	// Create a state with high alpha that might cause problems
	state := NewAircraftState()
	state.Altitude = 3000.0
	state.Position.Z = -3000.0
	state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
	state.Alpha = 30.0 * DEG_TO_RAD  // High angle of attack
	state.Beta = 0.0
	state.AngularRate = Vector3{X: 0.01, Y: 2.0, Z: 0.01}  // High pitch rate
	state.Controls.Elevator = 0.1
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("Test state:\n")
	fmt.Printf("   Alpha: %.1f°\n", state.Alpha*RAD_TO_DEG)
	fmt.Printf("   Angular rates: (%.3f, %.3f, %.3f) rad/s\n",
		state.AngularRate.X, state.AngularRate.Y, state.AngularRate.Z)
	fmt.Printf("   Controls: Elevator=%.3f\n", state.Controls.Elevator)
	
	// Calculate dynamic pressure
	q := 0.5 * state.Density * state.TrueAirspeed * state.TrueAirspeed
	qSc := q * calc.WingArea * calc.Chord
	
	fmt.Printf("\nMoment calculation details:\n")
	fmt.Printf("   Dynamic pressure: %.3f Pa\n", q)
	fmt.Printf("   qSc: %.3f N·m\n", qSc)
	
	// Individual moment components
	Cm0 := 0.05
	Cmalpha := -0.5
	Cmq := -8.0
	Cmde := -1.2
	
	term1 := Cm0
	term2 := Cmalpha * state.Alpha
	term3 := Cmq * state.AngularRate.Y
	term4 := Cmde * state.Controls.Elevator
	
	Cm := term1 + term2 + term3 + term4
	pitchMoment := Cm * qSc
	
	fmt.Printf("\nPitch moment breakdown:\n")
	fmt.Printf("   Cm0 term: %.6f\n", term1)
	fmt.Printf("   Cmalpha term: %.6f * %.3f = %.6f\n", Cmalpha, state.Alpha, term2)
	fmt.Printf("   Cmq term: %.6f * %.3f = %.6f\n", Cmq, state.AngularRate.Y, term3)
	fmt.Printf("   Cmde term: %.6f * %.3f = %.6f\n", Cmde, state.Controls.Elevator, term4)
	fmt.Printf("   Total Cm: %.6f\n", Cm)
	fmt.Printf("   Pitch moment: %.3f N·m\n", pitchMoment)
	
	// Angular acceleration
	pitchAccel := pitchMoment / calc.Inertia.YY
	fmt.Printf("   Pitch acceleration: %.3f rad/s² (%.1f°/s²)\n", 
		pitchAccel, pitchAccel*RAD_TO_DEG)
	
	if math.Abs(pitchAccel) > 10 {
		fmt.Printf("   ⚠️  Very high pitch acceleration!\n")
	}
	
	// Check for problematic values
	if math.IsNaN(pitchMoment) || math.IsInf(pitchMoment, 0) {
		fmt.Printf("   ❌ Invalid pitch moment!\n")
	}
	if math.IsNaN(pitchAccel) || math.IsInf(pitchAccel, 0) {
		fmt.Printf("   ❌ Invalid pitch acceleration!\n")
	}
}
