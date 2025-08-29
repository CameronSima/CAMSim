// Flight Control System Components
// Implements JSBSim-compatible flight control components

package main

import (
	"fmt"
	"math"
)

// ComponentProcessor defines the interface for all FCS components
type ComponentProcessor interface {
	// Execute processes inputs and returns the output value
	Execute(properties *PropertyManager, dt float64) float64
	
	// GetName returns the component name
	GetName() string
	
	// GetType returns the component type
	GetType() string
	
	// GetInputs returns the list of input property names
	GetInputs() []string
	
	// GetOutput returns the output property name
	GetOutput() string
	
	// Reset resets the component's internal state
	Reset()
	
	// SetRateGroup assigns the component to a rate group
	SetRateGroup(rateGroup string)
	
	// GetRateGroup returns the component's rate group
	GetRateGroup() string
}

// BaseComponent provides common functionality for all components
type BaseComponent struct {
	Name      string
	Type      string
	Inputs    []string
	Output    string
	RateGroup string
	Enabled   bool
}

// GetName returns the component name
func (bc *BaseComponent) GetName() string {
	return bc.Name
}

// GetType returns the component type
func (bc *BaseComponent) GetType() string {
	return bc.Type
}

// GetInputs returns the input property names
func (bc *BaseComponent) GetInputs() []string {
	return bc.Inputs
}

// GetOutput returns the output property name
func (bc *BaseComponent) GetOutput() string {
	return bc.Output
}

// SetRateGroup assigns the component to a rate group
func (bc *BaseComponent) SetRateGroup(rateGroup string) {
	bc.RateGroup = rateGroup
}

// GetRateGroup returns the component's rate group
func (bc *BaseComponent) GetRateGroup() string {
	return bc.RateGroup
}

// Reset is implemented by specific components
func (bc *BaseComponent) Reset() {
	// Base implementation does nothing
}

// =============================================================================
// ACTUATOR COMPONENT
// =============================================================================

// ActuatorComponent implements a realistic actuator with rate limiting and lag
type ActuatorComponent struct {
	BaseComponent
	
	// Configuration
	RateLimit        float64 // Maximum rate of change (units/sec)
	Lag              float64 // Time constant (seconds) - 0 means no lag
	HysteresisWidth  float64 // Hysteresis band width
	BiasValue        float64 // Bias offset
	
	// Internal state
	currentValue     float64 // Current output value
	targetValue      float64 // Target value after rate limiting
	previousInput    float64 // Previous input for hysteresis
	initialized      bool    // First execution flag
}

// NewActuatorComponent creates a new actuator component
func NewActuatorComponent(name, input, output string) *ActuatorComponent {
	return &ActuatorComponent{
		BaseComponent: BaseComponent{
			Name:    name,
			Type:    "ACTUATOR",
			Inputs:  []string{input},
			Output:  output,
			Enabled: true,
		},
		RateLimit:       math.Inf(1), // No limit by default
		Lag:             0.0,         // No lag by default
		HysteresisWidth: 0.0,         // No hysteresis by default
		BiasValue:       0.0,         // No bias by default
	}
}

// Execute processes the actuator dynamics
func (ac *ActuatorComponent) Execute(properties *PropertyManager, dt float64) float64 {
	if !ac.Enabled || len(ac.Inputs) == 0 {
		return ac.currentValue
	}
	
	// Get input value
	inputValue := properties.Get(ac.Inputs[0]) + ac.BiasValue
	
	// Apply hysteresis if configured
	if ac.HysteresisWidth > 0.0 {
		halfWidth := ac.HysteresisWidth / 2.0
		if math.Abs(inputValue-ac.previousInput) < halfWidth {
			inputValue = ac.previousInput // Stay in hysteresis band
		}
		ac.previousInput = inputValue
	}
	
	// Initialize on first run
	if !ac.initialized {
		ac.currentValue = 0.0  // Start from zero
		ac.targetValue = 0.0   // Start from zero
		ac.initialized = true
	}
	
	// Apply rate limiting
	if !math.IsInf(ac.RateLimit, 1) && ac.RateLimit > 0.0 {
		maxChange := ac.RateLimit * dt
		if inputValue > ac.targetValue+maxChange {
			ac.targetValue += maxChange
		} else if inputValue < ac.targetValue-maxChange {
			ac.targetValue -= maxChange
		} else {
			ac.targetValue = inputValue
		}
	} else {
		ac.targetValue = inputValue
	}
	
	// Apply lag filtering (first-order)
	if ac.Lag > 0.0 {
		alpha := dt / (ac.Lag + dt)
		ac.currentValue += alpha * (ac.targetValue - ac.currentValue)
	} else {
		ac.currentValue = ac.targetValue
	}
	
	// Set output property
	if ac.Output != "" {
		properties.Set(ac.Output, ac.currentValue)
	}
	
	return ac.currentValue
}

// Reset resets the actuator's internal state
func (ac *ActuatorComponent) Reset() {
	ac.currentValue = 0.0
	ac.targetValue = 0.0
	ac.previousInput = 0.0
	ac.initialized = false
}

// SetRateLimit configures the maximum rate of change
func (ac *ActuatorComponent) SetRateLimit(rateLimit float64) {
	ac.RateLimit = rateLimit
}

// SetLag configures the lag time constant
func (ac *ActuatorComponent) SetLag(lag float64) {
	ac.Lag = lag
}

// SetHysteresis configures the hysteresis width
func (ac *ActuatorComponent) SetHysteresis(width float64) {
	ac.HysteresisWidth = width
}

// =============================================================================
// LAG FILTER COMPONENT
// =============================================================================

// LagFilterComponent implements a first-order lag filter
type LagFilterComponent struct {
	BaseComponent
	
	// Configuration
	C1 float64 // Time constant (seconds)
	
	// Internal state
	output      float64
	initialized bool
}

// NewLagFilterComponent creates a new lag filter
func NewLagFilterComponent(name, input, output string, timeConstant float64) *LagFilterComponent {
	return &LagFilterComponent{
		BaseComponent: BaseComponent{
			Name:    name,
			Type:    "LAG_FILTER",
			Inputs:  []string{input},
			Output:  output,
			Enabled: true,
		},
		C1: timeConstant,
	}
}

// Execute processes the lag filter
func (lf *LagFilterComponent) Execute(properties *PropertyManager, dt float64) float64 {
	if !lf.Enabled || len(lf.Inputs) == 0 || lf.C1 <= 0.0 {
		input := properties.Get(lf.Inputs[0])
		properties.Set(lf.Output, input)
		return input
	}
	
	input := properties.Get(lf.Inputs[0])
	
	// Initialize on first run
	if !lf.initialized {
		lf.output = 0.0  // Start from zero
		lf.initialized = true
	}
	
	// First-order lag: output(n) = output(n-1) + dt/τ * (input - output(n-1))
	alpha := dt / (lf.C1 + dt)
	lf.output += alpha * (input - lf.output)
	
	// Set output property
	if lf.Output != "" {
		properties.Set(lf.Output, lf.output)
	}
	
	return lf.output
}

// Reset resets the filter's internal state
func (lf *LagFilterComponent) Reset() {
	lf.output = 0.0
	lf.initialized = false
}

// =============================================================================
// GAIN COMPONENT
// =============================================================================

// GainComponent implements a simple gain (multiplication)
type GainComponent struct {
	BaseComponent
	
	// Configuration
	Gain float64
}

// NewGainComponent creates a new gain component
func NewGainComponent(name, input, output string, gain float64) *GainComponent {
	return &GainComponent{
		BaseComponent: BaseComponent{
			Name:    name,
			Type:    "GAIN",
			Inputs:  []string{input},
			Output:  output,
			Enabled: true,
		},
		Gain: gain,
	}
}

// Execute processes the gain
func (gc *GainComponent) Execute(properties *PropertyManager, dt float64) float64 {
	if !gc.Enabled || len(gc.Inputs) == 0 {
		return 0.0
	}
	
	input := properties.Get(gc.Inputs[0])
	output := input * gc.Gain
	
	// Set output property
	if gc.Output != "" {
		properties.Set(gc.Output, output)
	}
	
	return output
}

// =============================================================================
// SUMMER COMPONENT
// =============================================================================

// SummerComponent implements a summer (addition/subtraction)
type SummerComponent struct {
	BaseComponent
	
	// Configuration - signs for each input (+1 for add, -1 for subtract)
	Signs []float64
	Bias  float64
}

// NewSummerComponent creates a new summer component
func NewSummerComponent(name string, inputs []string, output string) *SummerComponent {
	// Default to adding all inputs
	signs := make([]float64, len(inputs))
	for i := range signs {
		signs[i] = 1.0
	}
	
	return &SummerComponent{
		BaseComponent: BaseComponent{
			Name:    name,
			Type:    "SUMMER",
			Inputs:  inputs,
			Output:  output,
			Enabled: true,
		},
		Signs: signs,
		Bias:  0.0,
	}
}

// Execute processes the summer
func (sc *SummerComponent) Execute(properties *PropertyManager, dt float64) float64 {
	if !sc.Enabled {
		return 0.0
	}
	
	sum := sc.Bias
	
	// Add/subtract each input with its sign
	for i, input := range sc.Inputs {
		sign := 1.0
		if i < len(sc.Signs) {
			sign = sc.Signs[i]
		}
		sum += sign * properties.Get(input)
	}
	
	// Set output property
	if sc.Output != "" {
		properties.Set(sc.Output, sum)
	}
	
	return sum
}

// SetSigns configures the signs for each input
func (sc *SummerComponent) SetSigns(signs []float64) {
	sc.Signs = signs
}

// SetBias configures the bias value
func (sc *SummerComponent) SetBias(bias float64) {
	sc.Bias = bias
}

// =============================================================================
// CLIPPER COMPONENT
// =============================================================================

// ClipperComponent implements value clipping (limiting)
type ClipperComponent struct {
	BaseComponent
	
	// Configuration
	MinValue float64
	MaxValue float64
}

// NewClipperComponent creates a new clipper component
func NewClipperComponent(name, input, output string, minVal, maxVal float64) *ClipperComponent {
	return &ClipperComponent{
		BaseComponent: BaseComponent{
			Name:    name,
			Type:    "CLIPPER",
			Inputs:  []string{input},
			Output:  output,
			Enabled: true,
		},
		MinValue: minVal,
		MaxValue: maxVal,
	}
}

// Execute processes the clipper
func (cc *ClipperComponent) Execute(properties *PropertyManager, dt float64) float64 {
	if !cc.Enabled || len(cc.Inputs) == 0 {
		return 0.0
	}
	
	input := properties.Get(cc.Inputs[0])
	output := math.Max(cc.MinValue, math.Min(cc.MaxValue, input))
	
	// Set output property
	if cc.Output != "" {
		properties.Set(cc.Output, output)
	}
	
	return output
}

// =============================================================================
// SWITCH COMPONENT
// =============================================================================

// SwitchComponent implements conditional switching between values
type SwitchComponent struct {
	BaseComponent
	
	// Configuration
	TestProperty string    // Property to test
	TestValue    float64   // Value to compare against
	TestType     string    // "GT", "LT", "GE", "LE", "EQ", "NE"
	TrueValue    float64   // Value when test is true
	FalseValue   float64   // Value when test is false
	TrueInput    string    // Input property when test is true
	FalseInput   string    // Input property when test is false
}

// NewSwitchComponent creates a new switch component
func NewSwitchComponent(name, output string) *SwitchComponent {
	return &SwitchComponent{
		BaseComponent: BaseComponent{
			Name:    name,
			Type:    "SWITCH",
			Inputs:  []string{}, // Will be set based on configuration
			Output:  output,
			Enabled: true,
		},
		TestType:   "GT",
		TrueValue:  1.0,
		FalseValue: 0.0,
	}
}

// Execute processes the switch logic
func (sw *SwitchComponent) Execute(properties *PropertyManager, dt float64) float64 {
	if !sw.Enabled {
		return 0.0
	}
	
	// Evaluate test condition
	testResult := false
	if sw.TestProperty != "" {
		testVal := properties.Get(sw.TestProperty)
		
		switch sw.TestType {
		case "GT":
			testResult = testVal > sw.TestValue
		case "LT":
			testResult = testVal < sw.TestValue
		case "GE":
			testResult = testVal >= sw.TestValue
		case "LE":
			testResult = testVal <= sw.TestValue
		case "EQ":
			testResult = math.Abs(testVal-sw.TestValue) < 1e-10
		case "NE":
			testResult = math.Abs(testVal-sw.TestValue) >= 1e-10
		}
	}
	
	// Determine output value
	var output float64
	if testResult {
		if sw.TrueInput != "" {
			output = properties.Get(sw.TrueInput)
		} else {
			output = sw.TrueValue
		}
	} else {
		if sw.FalseInput != "" {
			output = properties.Get(sw.FalseInput)
		} else {
			output = sw.FalseValue
		}
	}
	
	// Set output property
	if sw.Output != "" {
		properties.Set(sw.Output, output)
	}
	
	return output
}

// SetTest configures the test condition
func (sw *SwitchComponent) SetTest(property string, testType string, value float64) {
	sw.TestProperty = property
	sw.TestType = testType
	sw.TestValue = value
}

// SetValues configures the output values
func (sw *SwitchComponent) SetValues(trueValue, falseValue float64) {
	sw.TrueValue = trueValue
	sw.FalseValue = falseValue
}

// SetInputs configures input properties for true/false conditions
func (sw *SwitchComponent) SetInputs(trueInput, falseInput string) {
	sw.TrueInput = trueInput
	sw.FalseInput = falseInput
	
	// Update inputs list
	sw.Inputs = []string{}
	if trueInput != "" {
		sw.Inputs = append(sw.Inputs, trueInput)
	}
	if falseInput != "" {
		sw.Inputs = append(sw.Inputs, falseInput)
	}
}

// String returns a string representation of the component
func ComponentToString(comp ComponentProcessor) string {
	return fmt.Sprintf("%s[%s]: %v → %s (rate_group: %s)",
		comp.GetName(),
		comp.GetType(),
		comp.GetInputs(),
		comp.GetOutput(),
		comp.GetRateGroup())
}
