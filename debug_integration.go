package main

import (
	"fmt"
	"math"
)

// DebugIntegrationStep performs a single integration step with detailed logging
func DebugIntegrationStep() {
	fmt.Println("🔍 Debugging Integration Step")
	fmt.Println("============================")
	
	// Create simplified flight dynamics engine
	engine := NewSimplifiedFlightDynamicsEngine(NewEulerIntegrator())
	
	// Initial state
	state := NewAircraftState()
	state.Altitude = 3000.0
	state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
	state.Controls.Throttle = 0.7
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("Initial State:\n")
	printStateDebug(state, "INITIAL")
	
	// Check if initial state has any NaN values
	if hasNaNValues(state) {
		fmt.Printf("❌ INITIAL STATE HAS NaN VALUES!\n")
		return
	}
	
	// Calculate forces and moments
	fmt.Printf("\n🔧 Calculating Forces and Moments...\n")
	components, err := engine.Calculator.CalculateSimplifiedForces(state)
	if err != nil {
		fmt.Printf("❌ Forces calculation error: %v\n", err)
		return
	}
	
	printForcesDebug(components, "FORCES")
	
	// Check forces for NaN
	if hasNaNInForces(components) {
		fmt.Printf("❌ FORCES HAVE NaN VALUES!\n")
		return
	}
	
	// Calculate state derivatives
	fmt.Printf("\n📐 Calculating State Derivatives...\n")
	derivatives := engine.Calculator.CalculateStateDerivatives(state, components)
	
	printDerivativesDebug(derivatives, "DERIVATIVES")
	
	// Check derivatives for NaN
	if hasNaNInDerivatives(derivatives) {
		fmt.Printf("❌ DERIVATIVES HAVE NaN VALUES!\n")
		return
	}
	
	// Perform integration step
	fmt.Printf("\n⚙️  Performing Integration Step...\n")
	dt := 0.01
	newState := engine.Integrator.Integrate(state, derivatives, dt)
	
	printStateDebug(newState, "AFTER_INTEGRATION")
	
	// Check new state for NaN
	if hasNaNValues(newState) {
		fmt.Printf("❌ NEW STATE HAS NaN VALUES AFTER INTEGRATION!\n")
		
		// Let's manually step through the Euler integration
		fmt.Printf("\n🔬 Manual Euler Integration Debug:\n")
		manualEulerDebug(state, derivatives, dt)
		return
	}
	
	fmt.Printf("✅ Integration step completed successfully!\n")
	
	// Compare changes
	fmt.Printf("\n📊 State Changes:\n")
	fmt.Printf("   Position: (%.6f, %.6f, %.6f) → (%.6f, %.6f, %.6f)\n",
		state.Position.X, state.Position.Y, state.Position.Z,
		newState.Position.X, newState.Position.Y, newState.Position.Z)
	fmt.Printf("   Velocity: (%.6f, %.6f, %.6f) → (%.6f, %.6f, %.6f)\n",
		state.Velocity.X, state.Velocity.Y, state.Velocity.Z,
		newState.Velocity.X, newState.Velocity.Y, newState.Velocity.Z)
	fmt.Printf("   Altitude: %.6f → %.6f\n", state.Altitude, newState.Altitude)
	fmt.Printf("   Speed: %.6f → %.6f\n", state.TrueAirspeed, newState.TrueAirspeed)
}

// printStateDebug prints detailed state information
func printStateDebug(state *AircraftState, label string) {
	fmt.Printf("[%s] State Debug:\n", label)
	fmt.Printf("   Time: %.6f\n", state.Time)
	fmt.Printf("   Position: (%.6f, %.6f, %.6f)\n", state.Position.X, state.Position.Y, state.Position.Z)
	fmt.Printf("   Orientation: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		state.Orientation.W, state.Orientation.X, state.Orientation.Y, state.Orientation.Z)
	fmt.Printf("   Velocity: (%.6f, %.6f, %.6f)\n", state.Velocity.X, state.Velocity.Y, state.Velocity.Z)
	fmt.Printf("   Angular Rate: (%.6f, %.6f, %.6f)\n", state.AngularRate.X, state.AngularRate.Y, state.AngularRate.Z)
	fmt.Printf("   Altitude: %.6f\n", state.Altitude)
	fmt.Printf("   Alpha: %.6f rad (%.2f°)\n", state.Alpha, state.Alpha*RAD_TO_DEG)
	fmt.Printf("   Beta: %.6f rad (%.2f°)\n", state.Beta, state.Beta*RAD_TO_DEG)
	fmt.Printf("   TrueAirspeed: %.6f\n", state.TrueAirspeed)
	fmt.Printf("   Density: %.6f\n", state.Density)
	fmt.Printf("   Pressure: %.6f\n", state.Pressure)
	fmt.Printf("   Temperature: %.6f\n", state.Temperature)
}

// printForcesDebug prints detailed forces information
func printForcesDebug(components *ForceMomentComponents, label string) {
	fmt.Printf("[%s] Forces Debug:\n", label)
	fmt.Printf("   Aerodynamic: Lift=%.6f, Drag=%.6f, Side=%.6f\n",
		components.Aerodynamic.Lift, components.Aerodynamic.Drag, components.Aerodynamic.Side)
	fmt.Printf("   Propulsion: Thrust=%.6f, Torque=%.6f\n",
		components.Propulsion.Thrust, components.Propulsion.Torque)
	fmt.Printf("   Gravity: (%.6f, %.6f, %.6f)\n",
		components.Gravity.Weight.X, components.Gravity.Weight.Y, components.Gravity.Weight.Z)
	fmt.Printf("   Moments: Roll=%.6f, Pitch=%.6f, Yaw=%.6f\n",
		components.Moments.Roll, components.Moments.Pitch, components.Moments.Yaw)
	fmt.Printf("   Total Force: (%.6f, %.6f, %.6f)\n",
		components.TotalForce.X, components.TotalForce.Y, components.TotalForce.Z)
	fmt.Printf("   Total Moment: (%.6f, %.6f, %.6f)\n",
		components.TotalMoment.X, components.TotalMoment.Y, components.TotalMoment.Z)
}

// printDerivativesDebug prints detailed derivatives information
func printDerivativesDebug(derivatives *StateDerivatives, label string) {
	fmt.Printf("[%s] Derivatives Debug:\n", label)
	fmt.Printf("   VelocityDot: (%.6f, %.6f, %.6f)\n",
		derivatives.VelocityDot.X, derivatives.VelocityDot.Y, derivatives.VelocityDot.Z)
	fmt.Printf("   AngularRateDot: (%.6f, %.6f, %.6f)\n",
		derivatives.AngularRateDot.X, derivatives.AngularRateDot.Y, derivatives.AngularRateDot.Z)
	fmt.Printf("   AltitudeDot: %.6f\n", derivatives.AltitudeDot)
	fmt.Printf("   MassDot: %.6f\n", derivatives.MassDot)
}

// hasNaNValues checks if any state values are NaN
func hasNaNValues(state *AircraftState) bool {
	return math.IsNaN(state.Time) ||
		math.IsNaN(state.Position.X) || math.IsNaN(state.Position.Y) || math.IsNaN(state.Position.Z) ||
		math.IsNaN(state.Orientation.W) || math.IsNaN(state.Orientation.X) || math.IsNaN(state.Orientation.Y) || math.IsNaN(state.Orientation.Z) ||
		math.IsNaN(state.Velocity.X) || math.IsNaN(state.Velocity.Y) || math.IsNaN(state.Velocity.Z) ||
		math.IsNaN(state.AngularRate.X) || math.IsNaN(state.AngularRate.Y) || math.IsNaN(state.AngularRate.Z) ||
		math.IsNaN(state.Altitude) || math.IsNaN(state.Alpha) || math.IsNaN(state.Beta) ||
		math.IsNaN(state.TrueAirspeed) || math.IsNaN(state.Density) || math.IsNaN(state.Pressure) || math.IsNaN(state.Temperature)
}

// hasNaNInForces checks if any force values are NaN
func hasNaNInForces(components *ForceMomentComponents) bool {
	return math.IsNaN(components.Aerodynamic.Lift) || math.IsNaN(components.Aerodynamic.Drag) || math.IsNaN(components.Aerodynamic.Side) ||
		math.IsNaN(components.Propulsion.Thrust) || math.IsNaN(components.Propulsion.Torque) ||
		math.IsNaN(components.Gravity.Weight.X) || math.IsNaN(components.Gravity.Weight.Y) || math.IsNaN(components.Gravity.Weight.Z) ||
		math.IsNaN(components.Moments.Roll) || math.IsNaN(components.Moments.Pitch) || math.IsNaN(components.Moments.Yaw) ||
		math.IsNaN(components.TotalForce.X) || math.IsNaN(components.TotalForce.Y) || math.IsNaN(components.TotalForce.Z) ||
		math.IsNaN(components.TotalMoment.X) || math.IsNaN(components.TotalMoment.Y) || math.IsNaN(components.TotalMoment.Z)
}

// hasNaNInDerivatives checks if any derivative values are NaN
func hasNaNInDerivatives(derivatives *StateDerivatives) bool {
	return math.IsNaN(derivatives.VelocityDot.X) || math.IsNaN(derivatives.VelocityDot.Y) || math.IsNaN(derivatives.VelocityDot.Z) ||
		math.IsNaN(derivatives.AngularRateDot.X) || math.IsNaN(derivatives.AngularRateDot.Y) || math.IsNaN(derivatives.AngularRateDot.Z) ||
		math.IsNaN(derivatives.AltitudeDot) || math.IsNaN(derivatives.MassDot)
}

// manualEulerDebug manually steps through Euler integration to find the NaN source
func manualEulerDebug(state *AircraftState, derivatives *StateDerivatives, dt float64) {
	fmt.Printf("Manual Euler Integration Debug:\n")
	
	// Copy initial state
	newState := state.Copy()
	fmt.Printf("   Copied state: OK\n")
	
	// Advance time
	newState.Time += dt
	fmt.Printf("   Time: %.6f → %.6f\n", state.Time, newState.Time)
	
	// Transform body velocity to Earth frame for position integration
	earthVel := state.Orientation.RotateVector(state.Velocity)
	fmt.Printf("   Earth velocity: (%.6f, %.6f, %.6f)\n", earthVel.X, earthVel.Y, earthVel.Z)
	
	// Check for NaN in earth velocity
	if math.IsNaN(earthVel.X) || math.IsNaN(earthVel.Y) || math.IsNaN(earthVel.Z) {
		fmt.Printf("   ❌ NaN in earth velocity after rotation!\n")
		fmt.Printf("   Checking quaternion rotation...\n")
		debugQuaternionRotation(state.Orientation, state.Velocity)
		return
	}
	
	// Integrate position
	positionDelta := earthVel.Scale(dt)
	fmt.Printf("   Position delta: (%.6f, %.6f, %.6f)\n", positionDelta.X, positionDelta.Y, positionDelta.Z)
	
	newState.Position = state.Position.Add(positionDelta)
	fmt.Printf("   New position: (%.6f, %.6f, %.6f)\n", newState.Position.X, newState.Position.Y, newState.Position.Z)
	
	// Check for NaN after position integration
	if math.IsNaN(newState.Position.X) || math.IsNaN(newState.Position.Y) || math.IsNaN(newState.Position.Z) {
		fmt.Printf("   ❌ NaN in position after integration!\n")
		return
	}
	
	// Integrate orientation (quaternion)
	omegaQuat := Quaternion{W: 0, X: state.AngularRate.X, Y: state.AngularRate.Y, Z: state.AngularRate.Z}
	fmt.Printf("   Omega quaternion: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		omegaQuat.W, omegaQuat.X, omegaQuat.Y, omegaQuat.Z)
	
	orientationDot := state.Orientation.Multiply(omegaQuat).Scale(0.5)
	fmt.Printf("   Orientation dot: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		orientationDot.W, orientationDot.X, orientationDot.Y, orientationDot.Z)
	
	// Check for NaN in orientation dot
	if math.IsNaN(orientationDot.W) || math.IsNaN(orientationDot.X) || math.IsNaN(orientationDot.Y) || math.IsNaN(orientationDot.Z) {
		fmt.Printf("   ❌ NaN in orientation dot after quaternion multiply!\n")
		debugQuaternionMultiply(state.Orientation, omegaQuat)
		return
	}
	
	orientationDelta := orientationDot.Scale(dt)
	newState.Orientation = state.Orientation.Add(orientationDelta).Normalize()
	fmt.Printf("   New orientation: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n",
		newState.Orientation.W, newState.Orientation.X, newState.Orientation.Y, newState.Orientation.Z)
	
	// Check for NaN after orientation integration
	if math.IsNaN(newState.Orientation.W) || math.IsNaN(newState.Orientation.X) || math.IsNaN(newState.Orientation.Y) || math.IsNaN(newState.Orientation.Z) {
		fmt.Printf("   ❌ NaN in orientation after integration!\n")
		return
	}
	
	// Integrate linear velocity
	velocityDelta := derivatives.VelocityDot.Scale(dt)
	fmt.Printf("   Velocity delta: (%.6f, %.6f, %.6f)\n", velocityDelta.X, velocityDelta.Y, velocityDelta.Z)
	
	newState.Velocity = state.Velocity.Add(velocityDelta)
	fmt.Printf("   New velocity: (%.6f, %.6f, %.6f)\n", newState.Velocity.X, newState.Velocity.Y, newState.Velocity.Z)
	
	// Integrate angular velocity
	angularDelta := derivatives.AngularRateDot.Scale(dt)
	newState.AngularRate = state.AngularRate.Add(angularDelta)
	fmt.Printf("   New angular rate: (%.6f, %.6f, %.6f)\n", newState.AngularRate.X, newState.AngularRate.Y, newState.AngularRate.Z)
	
	// Update altitude
	newState.Altitude = -newState.Position.Z
	fmt.Printf("   New altitude: %.6f\n", newState.Altitude)
	
	// Update derived parameters
	fmt.Printf("   Updating atmosphere...\n")
	newState.UpdateAtmosphere()
	
	fmt.Printf("   Updating derived parameters...\n")
	newState.UpdateDerivedParameters()
	
	fmt.Printf("   ✅ Manual integration completed successfully!\n")
}

// debugQuaternionRotation checks quaternion vector rotation for NaN issues
func debugQuaternionRotation(q Quaternion, v Vector3) {
	fmt.Printf("   Debugging quaternion rotation:\n")
	fmt.Printf("     Quaternion: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n", q.W, q.X, q.Y, q.Z)
	fmt.Printf("     Vector: (%.6f, %.6f, %.6f)\n", v.X, v.Y, v.Z)
	
	// Check quaternion normalization
	qMag := math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
	fmt.Printf("     Quaternion magnitude: %.6f\n", qMag)
	
	if math.IsNaN(qMag) || qMag < 1e-10 {
		fmt.Printf("     ❌ Invalid quaternion magnitude!\n")
		return
	}
	
	// Manual quaternion-vector rotation: v' = q * v * q*
	// First, convert vector to quaternion
	vQuat := Quaternion{W: 0, X: v.X, Y: v.Y, Z: v.Z}
	
	// Quaternion conjugate
	qConj := Quaternion{W: q.W, X: -q.X, Y: -q.Y, Z: -q.Z}
	
	// q * v
	qv := q.Multiply(vQuat)
	fmt.Printf("     q * v: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n", qv.W, qv.X, qv.Y, qv.Z)
	
	// (q * v) * q*
	result := qv.Multiply(qConj)
	fmt.Printf("     Result: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n", result.W, result.X, result.Y, result.Z)
}

// debugQuaternionMultiply checks quaternion multiplication for NaN issues
func debugQuaternionMultiply(q1, q2 Quaternion) {
	fmt.Printf("   Debugging quaternion multiplication:\n")
	fmt.Printf("     Q1: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n", q1.W, q1.X, q1.Y, q1.Z)
	fmt.Printf("     Q2: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n", q2.W, q2.X, q2.Y, q2.Z)
	
	// Manual quaternion multiplication
	w := q1.W*q2.W - q1.X*q2.X - q1.Y*q2.Y - q1.Z*q2.Z
	x := q1.W*q2.X + q1.X*q2.W + q1.Y*q2.Z - q1.Z*q2.Y
	y := q1.W*q2.Y - q1.X*q2.Z + q1.Y*q2.W + q1.Z*q2.X
	z := q1.W*q2.Z + q1.X*q2.Y - q1.Y*q2.X + q1.Z*q2.W
	
	fmt.Printf("     Result: (W=%.6f, X=%.6f, Y=%.6f, Z=%.6f)\n", w, x, y, z)
	
	if math.IsNaN(w) || math.IsNaN(x) || math.IsNaN(y) || math.IsNaN(z) {
		fmt.Printf("     ❌ NaN in quaternion multiplication result!\n")
	}
}

// DebugMultipleSteps runs multiple integration steps to see when NaN first appears
func DebugMultipleSteps() {
	fmt.Println("\n🔍 Debugging Multiple Integration Steps")
	fmt.Println("======================================")
	
	engine := NewSimplifiedFlightDynamicsEngine(NewEulerIntegrator())
	
	state := NewAircraftState()
	state.Altitude = 3000.0
	state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
	state.Controls.Throttle = 0.7
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	dt := 0.01
	maxSteps := 10
	
	fmt.Printf("Starting with valid state...\n")
	
	for step := 0; step < maxSteps; step++ {
		fmt.Printf("\n--- Step %d ---\n", step+1)
		
		// Check state before step
		if hasNaNValues(state) {
			fmt.Printf("❌ NaN detected in state BEFORE step %d!\n", step+1)
			printStateDebug(state, "NaN_STATE")
			break
		}
		
		// Take one step
		newState, err := engine.Step(state, dt)
		if err != nil {
			fmt.Printf("❌ Error in step %d: %v\n", step+1, err)
			break
		}
		
		// Check state after step
		if hasNaNValues(newState) {
			fmt.Printf("❌ NaN detected AFTER step %d!\n", step+1)
			fmt.Printf("State before step:\n")
			printStateDebug(state, "BEFORE_NaN")
			fmt.Printf("State after step:\n")
			printStateDebug(newState, "AFTER_NaN")
			
			// Debug this specific step
			fmt.Printf("\nDebugging the problematic step...\n")
			DebugIntegrationStep()
			break
		}
		
		fmt.Printf("✅ Step %d completed successfully\n", step+1)
		fmt.Printf("   Position: (%.3f, %.3f, %.3f)\n", newState.Position.X, newState.Position.Y, newState.Position.Z)
		fmt.Printf("   Velocity: (%.3f, %.3f, %.3f)\n", newState.Velocity.X, newState.Velocity.Y, newState.Velocity.Z)
		fmt.Printf("   Speed: %.3f m/s\n", newState.TrueAirspeed)
		
		state = newState
	}
}
