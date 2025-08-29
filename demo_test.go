package main

import (
	"testing"
)

// TestFlightDynamicsDemo tests the complete flight dynamics demonstration
func TestFlightDynamicsDemo(t *testing.T) {
	// Just run the demo to ensure it doesn't crash
	FlightDynamicsDemo()
	StallDemo()
}
