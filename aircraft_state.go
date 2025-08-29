package main

import (
	"fmt"
	"math"
	"time"
)

// Vector3 represents a 3D vector for position, velocity, acceleration, etc.
type Vector3 struct {
	X, Y, Z float64
}

// Add adds two vectors
func (v Vector3) Add(other Vector3) Vector3 {
	return Vector3{X: v.X + other.X, Y: v.Y + other.Y, Z: v.Z + other.Z}
}

// Scale multiplies vector by a scalar
func (v Vector3) Scale(scalar float64) Vector3 {
	return Vector3{X: v.X * scalar, Y: v.Y * scalar, Z: v.Z * scalar}
}

// Magnitude returns the magnitude of the vector
func (v Vector3) Magnitude() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

// Normalize returns a unit vector in the same direction
func (v Vector3) Normalize() Vector3 {
	mag := v.Magnitude()
	if mag == 0 {
		return Vector3{0, 0, 0}
	}
	return Vector3{X: v.X / mag, Y: v.Y / mag, Z: v.Z / mag}
}

// Dot returns the dot product of two vectors
func (v Vector3) Dot(other Vector3) float64 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

// Cross returns the cross product of two vectors
func (v Vector3) Cross(other Vector3) Vector3 {
	return Vector3{
		X: v.Y*other.Z - v.Z*other.Y,
		Y: v.Z*other.X - v.X*other.Z,
		Z: v.X*other.Y - v.Y*other.X,
	}
}

// Quaternion represents rotation using quaternions for smooth interpolation
type Quaternion struct {
	W, X, Y, Z float64
}

// NewQuaternionFromEuler creates a quaternion from Euler angles (roll, pitch, yaw in radians)
func NewQuaternionFromEuler(roll, pitch, yaw float64) Quaternion {
	cr := math.Cos(roll * 0.5)
	sr := math.Sin(roll * 0.5)
	cp := math.Cos(pitch * 0.5)
	sp := math.Sin(pitch * 0.5)
	cy := math.Cos(yaw * 0.5)
	sy := math.Sin(yaw * 0.5)

	return Quaternion{
		W: cr*cp*cy + sr*sp*sy,
		X: sr*cp*cy - cr*sp*sy,
		Y: cr*sp*cy + sr*cp*sy,
		Z: cr*cp*sy - sr*sp*cy,
	}
}

// ToEuler converts quaternion to Euler angles (roll, pitch, yaw in radians)
func (q Quaternion) ToEuler() (roll, pitch, yaw float64) {
	// Roll (x-axis rotation)
	sinr_cosp := 2 * (q.W*q.X + q.Y*q.Z)
	cosr_cosp := 1 - 2*(q.X*q.X+q.Y*q.Y)
	roll = math.Atan2(sinr_cosp, cosr_cosp)

	// Pitch (y-axis rotation)
	sinp := 2 * (q.W*q.Y - q.Z*q.X)
	if math.Abs(sinp) >= 1 {
		if sinp >= 0 {
			pitch = math.Pi / 2
		} else {
			pitch = -math.Pi / 2
		}
	} else {
		pitch = math.Asin(sinp)
	}

	// Yaw (z-axis rotation)
	siny_cosp := 2 * (q.W*q.Z + q.X*q.Y)
	cosy_cosp := 1 - 2*(q.Y*q.Y+q.Z*q.Z)
	yaw = math.Atan2(siny_cosp, cosy_cosp)

	return roll, pitch, yaw
}

// Normalize normalizes the quaternion
func (q Quaternion) Normalize() Quaternion {
	norm := math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
	if norm == 0 {
		return Quaternion{W: 1, X: 0, Y: 0, Z: 0}
	}
	return Quaternion{
		W: q.W / norm,
		X: q.X / norm,
		Y: q.Y / norm,
		Z: q.Z / norm,
	}
}

// RotateVector rotates a vector by this quaternion
func (q Quaternion) RotateVector(v Vector3) Vector3 {
	// v' = q * v * q^-1
	qv := Quaternion{W: 0, X: v.X, Y: v.Y, Z: v.Z}
	qConj := Quaternion{W: q.W, X: -q.X, Y: -q.Y, Z: -q.Z}
	
	// q * v
	temp := q.Multiply(qv)
	// (q * v) * q^-1
	result := temp.Multiply(qConj)
	
	return Vector3{X: result.X, Y: result.Y, Z: result.Z}
}

// Multiply multiplies two quaternions
func (q Quaternion) Multiply(other Quaternion) Quaternion {
	return Quaternion{
		W: q.W*other.W - q.X*other.X - q.Y*other.Y - q.Z*other.Z,
		X: q.W*other.X + q.X*other.W + q.Y*other.Z - q.Z*other.Y,
		Y: q.W*other.Y - q.X*other.Z + q.Y*other.W + q.Z*other.X,
		Z: q.W*other.Z + q.X*other.Y - q.Y*other.X + q.Z*other.W,
	}
}

// ControlInputs represents pilot control inputs
type ControlInputs struct {
	Aileron   float64 `json:"aileron"`   // Roll control: -1 (full left) to +1 (full right)
	Elevator  float64 `json:"elevator"`  // Pitch control: -1 (full down) to +1 (full up)
	Rudder    float64 `json:"rudder"`    // Yaw control: -1 (full left) to +1 (full right)
	Throttle  float64 `json:"throttle"`  // Engine power: 0 (idle) to 1 (full power)
	Flaps     float64 `json:"flaps"`     // Flap position: 0 (retracted) to 1 (full extended)
	Gear      bool    `json:"gear"`      // Landing gear: true = down, false = up
	Brake     float64 `json:"brake"`     // Brake pressure: 0 to 1
	Mixture   float64 `json:"mixture"`   // Fuel mixture: 0 (lean) to 1 (rich)
	Propeller float64 `json:"propeller"` // Propeller pitch: 0 to 1 (for variable pitch props)
}

// NewControlInputs creates control inputs with safe defaults
func NewControlInputs() ControlInputs {
	return ControlInputs{
		Aileron:   0.0,
		Elevator:  0.0,
		Rudder:    0.0,
		Throttle:  0.0,
		Flaps:     0.0,
		Gear:      true,  // Start with gear down for safety
		Brake:     0.0,
		Mixture:   0.8,   // Start rich for engine start
		Propeller: 1.0,   // Start at fine pitch
	}
}

// AircraftState represents the complete state of the aircraft at any given time
type AircraftState struct {
	// Time
	Time        float64   `json:"time"`         // Simulation time in seconds
	Timestamp   time.Time `json:"timestamp"`    // Real-world timestamp
	
	// Position (Earth-fixed coordinates)
	Position    Vector3   `json:"position"`     // X, Y, Z in meters (NED: North, East, Down)
	Latitude    float64   `json:"latitude"`     // Latitude in radians
	Longitude   float64   `json:"longitude"`    // Longitude in radians
	Altitude    float64   `json:"altitude"`     // Altitude above sea level in meters
	
	// Orientation (Body frame relative to Earth frame)
	Orientation Quaternion `json:"orientation"` // Attitude quaternion
	Roll        float64    `json:"roll"`        // Roll angle in radians
	Pitch       float64    `json:"pitch"`       // Pitch angle in radians
	Yaw         float64    `json:"yaw"`         // Yaw angle (heading) in radians
	
	// Linear Motion (Body frame)
	Velocity     Vector3 `json:"velocity"`      // u, v, w in m/s (forward, right, down)
	Acceleration Vector3 `json:"acceleration"`  // Linear acceleration in m/s²
	
	// Angular Motion (Body frame)
	AngularRate  Vector3 `json:"angular_rate"`  // p, q, r in rad/s (roll, pitch, yaw rates)
	AngularAccel Vector3 `json:"angular_accel"` // Angular acceleration in rad/s²
	
	// Flight Parameters (derived from state)
	Alpha         float64 `json:"alpha"`          // Angle of attack in radians
	Beta          float64 `json:"beta"`           // Sideslip angle in radians
	Mach          float64 `json:"mach"`           // Mach number
	IndicatedAirspeed float64 `json:"ias"`        // Indicated airspeed in m/s
	TrueAirspeed     float64 `json:"tas"`         // True airspeed in m/s
	CalibratedAirspeed float64 `json:"cas"`       // Calibrated airspeed in m/s
	GroundSpeed      float64 `json:"groundspeed"` // Ground speed in m/s
	
	// Atmospheric Conditions
	Temperature   float64 `json:"temperature"`    // Air temperature in Kelvin
	Pressure      float64 `json:"pressure"`       // Static pressure in Pa
	Density       float64 `json:"density"`        // Air density in kg/m³
	SoundSpeed    float64 `json:"sound_speed"`    // Speed of sound in m/s
	DynamicPressure float64 `json:"qbar"`         // Dynamic pressure in Pa
	
	// Control Surface Positions (actual positions, may differ from inputs due to limits/delays)
	ControlSurfaces struct {
		AileronLeft   float64 `json:"aileron_left"`   // Left aileron position
		AileronRight  float64 `json:"aileron_right"`  // Right aileron position
		Elevator      float64 `json:"elevator"`       // Elevator position
		Rudder        float64 `json:"rudder"`         // Rudder position
		FlapLeft      float64 `json:"flap_left"`      // Left flap position
		FlapRight     float64 `json:"flap_right"`     // Right flap position
		Trim          struct {
			Aileron  float64 `json:"aileron"`   // Aileron trim
			Elevator float64 `json:"elevator"`  // Elevator trim
			Rudder   float64 `json:"rudder"`    // Rudder trim
		} `json:"trim"`
	} `json:"control_surfaces"`
	
	// Propulsion
	Engine struct {
		Running    bool    `json:"running"`     // Engine running state
		RPM        float64 `json:"rpm"`         // Engine RPM
		ManifoldP  float64 `json:"manifold_p"`  // Manifold pressure
		FuelFlow   float64 `json:"fuel_flow"`   // Fuel flow rate
		EGT        float64 `json:"egt"`         // Exhaust gas temperature
		CHT        float64 `json:"cht"`         // Cylinder head temperature
		OilTemp    float64 `json:"oil_temp"`    // Oil temperature
		OilPress   float64 `json:"oil_press"`   // Oil pressure
		Thrust     float64 `json:"thrust"`      // Current thrust in Newtons
	} `json:"engine"`
	
	// Landing Gear
	Gear struct {
		Down         bool    `json:"down"`          // Gear down/up
		Transition   float64 `json:"transition"`    // Transition state (0=up, 1=down)
		OnGround     bool    `json:"on_ground"`     // Aircraft on ground
		GroundHeight float64 `json:"ground_height"` // Height of ground below aircraft
		Compression  struct {
			Main  float64 `json:"main"`   // Main gear compression
			Nose  float64 `json:"nose"`   // Nose gear compression
		} `json:"compression"`
	} `json:"gear"`
	
	// Forces and Moments (for analysis/debugging)
	Forces struct {
		Aerodynamic Vector3 `json:"aerodynamic"` // Aerodynamic forces in body frame
		Propulsive  Vector3 `json:"propulsive"`  // Propulsive forces in body frame
		Gravity     Vector3 `json:"gravity"`     // Gravitational force in body frame
		Total       Vector3 `json:"total"`       // Total forces in body frame
	} `json:"forces"`
	
	Moments struct {
		Aerodynamic Vector3 `json:"aerodynamic"` // Aerodynamic moments in body frame
		Propulsive  Vector3 `json:"propulsive"`  // Propulsive moments in body frame
		Gyroscopic  Vector3 `json:"gyroscopic"`  // Gyroscopic moments
		Total       Vector3 `json:"total"`       // Total moments in body frame
	} `json:"moments"`
	
	// Control Inputs (pilot commands)
	Controls ControlInputs `json:"controls"`
}

// Additional constants for aircraft state (others defined in jsbsimxmlparser.go)
const (
	KT_TO_MS   = 0.514444
	MS_TO_KT   = 1.0 / KT_TO_MS
	LB_TO_N    = 4.44822
	N_TO_LB    = 1.0 / LB_TO_N
)

// NewAircraftState creates a new aircraft state with sensible defaults
func NewAircraftState() *AircraftState {
	state := &AircraftState{
		Time:        0.0,
		Timestamp:   time.Now(),
		Position:    Vector3{X: 0, Y: 0, Z: 0},
		Latitude:    0.0,
		Longitude:   0.0,
		Altitude:    1000.0, // Start at 1000m AGL
		Orientation: NewQuaternionFromEuler(0, 0, 0), // Level flight
		Roll:        0.0,
		Pitch:       0.0,
		Yaw:         0.0,
		Velocity:    Vector3{X: 50.0, Y: 0, Z: 0}, // 50 m/s forward (about 100 knots)
		Controls:    NewControlInputs(),
	}
	
	// Initialize atmospheric conditions at 1000m
	state.UpdateAtmosphere()
	
	// Calculate derived parameters
	state.UpdateDerivedParameters()
	
	return state
}

// UpdateAtmosphere updates atmospheric conditions based on altitude (ISA Standard Atmosphere)
func (state *AircraftState) UpdateAtmosphere() {
	// ISA Standard Atmosphere model
	const (
		seaLevelTemp     = 288.15    // K (15°C)
		seaLevelPressure = 101325.0  // Pa
		seaLevelDensity  = 1.225     // kg/m³
		tempLapseRate    = 0.0065    // K/m
		gasConstant      = 287.05    // J/(kg·K)
		gamma            = 1.4       // Specific heat ratio for air
	)
	
	altitude := state.Altitude
	if altitude < 0 {
		altitude = 0 // Don't go below sea level for atmosphere calculation
	}
	
	// Temperature (linear decrease up to 11,000m)
	if altitude <= 11000 {
		state.Temperature = seaLevelTemp - tempLapseRate*altitude
	} else {
		state.Temperature = 216.65 // Constant above 11km
	}
	
	// Pressure (hydrostatic equation)
	if altitude <= 11000 {
		state.Pressure = seaLevelPressure * math.Pow(state.Temperature/seaLevelTemp, 9.80665/(gasConstant*tempLapseRate))
	} else {
		p11 := seaLevelPressure * math.Pow(216.65/seaLevelTemp, 9.80665/(gasConstant*tempLapseRate))
		state.Pressure = p11 * math.Exp(-9.80665*(altitude-11000)/(gasConstant*216.65))
	}
	
	// Density (ideal gas law)
	state.Density = state.Pressure / (gasConstant * state.Temperature)
	
	// Speed of sound
	state.SoundSpeed = math.Sqrt(gamma * gasConstant * state.Temperature)
	
	// Dynamic pressure
	state.DynamicPressure = 0.5 * state.Density * state.TrueAirspeed * state.TrueAirspeed
}

// UpdateDerivedParameters calculates derived flight parameters from basic state
func (state *AircraftState) UpdateDerivedParameters() {
	// Update Euler angles from quaternion
	state.Roll, state.Pitch, state.Yaw = state.Orientation.ToEuler()
	
	// Calculate airspeed components
	state.TrueAirspeed = state.Velocity.Magnitude()
	
	// Indicated airspeed (simplified - assumes no instrument error)
	// IAS = TAS * sqrt(density / sea_level_density)
	state.IndicatedAirspeed = state.TrueAirspeed * math.Sqrt(state.Density/1.225)
	
	// Calibrated airspeed (simplified - same as IAS for now)
	state.CalibratedAirspeed = state.IndicatedAirspeed
	
	// Mach number
	if state.SoundSpeed > 0 {
		state.Mach = state.TrueAirspeed / state.SoundSpeed
	}
	
	// Angle of attack (alpha) - angle between velocity and body X axis
	if state.Velocity.X != 0 || state.Velocity.Z != 0 {
		state.Alpha = math.Atan2(-state.Velocity.Z, state.Velocity.X)
	}
	
	// Sideslip angle (beta) - angle between velocity and XZ plane
	if state.TrueAirspeed > 0 {
		state.Beta = math.Asin(state.Velocity.Y / state.TrueAirspeed)
	}
	
	// Ground speed (simplified - magnitude of horizontal velocity)
	groundVel := Vector3{X: state.Velocity.X, Y: state.Velocity.Y, Z: 0}
	state.GroundSpeed = groundVel.Magnitude()
	
	// Update dynamic pressure
	state.DynamicPressure = 0.5 * state.Density * state.TrueAirspeed * state.TrueAirspeed
}

// Copy creates a deep copy of the aircraft state
func (state *AircraftState) Copy() *AircraftState {
	newState := *state // Shallow copy
	return &newState   // Return pointer to the copy
}

// ToPropertyMap converts the aircraft state to a property map for function evaluation
func (state *AircraftState) ToPropertyMap() map[string]float64 {
	return map[string]float64{
		// Position and orientation
		"position/latitude-rad":     state.Latitude,
		"position/longitude-rad":    state.Longitude,
		"position/h-sl-m":          state.Altitude,
		"position/h-agl-m":         state.Altitude - state.Gear.GroundHeight,
		"attitude/roll-rad":        state.Roll,
		"attitude/pitch-rad":       state.Pitch,
		"attitude/heading-rad":     state.Yaw,
		
		// Velocities and rates
		"velocities/u-mps":         state.Velocity.X,
		"velocities/v-mps":         state.Velocity.Y,
		"velocities/w-mps":         state.Velocity.Z,
		"velocities/p-rad_sec":     state.AngularRate.X,
		"velocities/q-rad_sec":     state.AngularRate.Y,
		"velocities/r-rad_sec":     state.AngularRate.Z,
		"velocities/vt-mps":        state.TrueAirspeed,
		"velocities/vc-mps":        state.CalibratedAirspeed,
		"velocities/vi-mps":        state.IndicatedAirspeed,
		
		// Flight parameters
		"aero/alpha-rad":           state.Alpha,
		"aero/beta-rad":            state.Beta,
		"aero/alpha-deg":           state.Alpha * RAD_TO_DEG,
		"aero/beta-deg":            state.Beta * RAD_TO_DEG,
		"aero/mach":                state.Mach,
		"aero/qbar-Pa":             state.DynamicPressure,
		"aero/qbar-psf":            state.DynamicPressure * 0.020885, // Pa to psf conversion
		
		// Atmospheric conditions
		"atmosphere/T-K":           state.Temperature,
		"atmosphere/P-Pa":          state.Pressure,
		"atmosphere/rho-kgm3":      state.Density,
		"atmosphere/rho-slugs_ft3": state.Density * 0.00194032, // kg/m³ to slugs/ft³
		"atmosphere/a-mps":         state.SoundSpeed,
		
		// Controls
		"fcs/aileron-cmd-norm":     state.Controls.Aileron,
		"fcs/elevator-cmd-norm":    state.Controls.Elevator,
		"fcs/rudder-cmd-norm":      state.Controls.Rudder,
		"fcs/throttle-cmd-norm":    state.Controls.Throttle,
		"fcs/flap-cmd-norm":        state.Controls.Flaps,
		"fcs/gear-cmd-norm":        func() float64 { if state.Controls.Gear { return 1.0 } else { return 0.0 } }(),
		
		// Control surfaces (actual positions)
		"fcs/left-aileron-pos-rad":  state.ControlSurfaces.AileronLeft,
		"fcs/right-aileron-pos-rad": state.ControlSurfaces.AileronRight,
		"fcs/elevator-pos-rad":      state.ControlSurfaces.Elevator,
		"fcs/rudder-pos-rad":        state.ControlSurfaces.Rudder,
		"fcs/flap-pos-deg":          state.ControlSurfaces.FlapLeft * RAD_TO_DEG,
		
		// Engine
		"propulsion/engine/thrust-N":    state.Engine.Thrust,
		"propulsion/engine/thrust-lbs":  state.Engine.Thrust * N_TO_LB,
		"propulsion/engine/power-hp":    state.Engine.Thrust * state.TrueAirspeed / 745.7, // Rough conversion
		"engines/engine/rpm":            state.Engine.RPM,
		"engines/engine/mp-inHg":        state.Engine.ManifoldP,
		
		// Landing gear
		"gear/gear-down":               func() float64 { if state.Gear.Down { return 1.0 } else { return 0.0 } }(),
		"gear/gear-pos-norm":           state.Gear.Transition,
		"gear/wow":                     func() float64 { if state.Gear.OnGround { return 1.0 } else { return 0.0 } }(),
		
		// Forces and moments (for analysis)
		"forces/fbx-N":                 state.Forces.Total.X,
		"forces/fby-N":                 state.Forces.Total.Y,
		"forces/fbz-N":                 state.Forces.Total.Z,
		"moments/l-Nm":                 state.Moments.Total.X,
		"moments/m-Nm":                 state.Moments.Total.Y,
		"moments/n-Nm":                 state.Moments.Total.Z,
		
		// Time
		"simulation/sim-time-sec":      state.Time,
	}
}

// SetControlInputs updates only the control inputs in the aircraft state
// For control surface positioning, use FlightDynamicsEngineWithFCS which processes
// control inputs through the Flight Control System (FCS) with proper dynamics.
func (state *AircraftState) SetControlInputs(controls ControlInputs) {
	state.Controls = controls
	
	// Update gear state (gear is typically not processed through FCS)
	state.Gear.Down = controls.Gear
	if controls.Gear {
		state.Gear.Transition = 1.0
	} else {
		state.Gear.Transition = 0.0
	}
}

// String returns a formatted string representation of key state parameters
func (state *AircraftState) String() string {
	return fmt.Sprintf(
		"T=%.1fs Alt=%.0fm IAS=%.1fkt α=%.1f° β=%.1f° φ=%.1f° θ=%.1f° ψ=%.1f°",
		state.Time,
		state.Altitude,
		state.IndicatedAirspeed*MS_TO_KT,
		state.Alpha*RAD_TO_DEG,
		state.Beta*RAD_TO_DEG,
		state.Roll*RAD_TO_DEG,
		state.Pitch*RAD_TO_DEG,
		state.Yaw*RAD_TO_DEG,
	)
}
