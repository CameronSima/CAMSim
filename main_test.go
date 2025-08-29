package main

import (
	"os"
	"strings"
	"testing"
)

func TestExtractAllValuesWithRealP51DFile(t *testing.T) {
	// Open the actual P-51D XML file
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Fatalf("Failed to open P-51D XML file: %v", err)
	}
	defer file.Close()

	// Parse the configuration
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Fatalf("Failed to parse P-51D XML: %v", err)
	}

	// Extract all values
	values := ExtractAllValues(config)

	// Test some key values from the actual file
	t.Run("Header Values", func(t *testing.T) {
		if author, ok := values["header.author"].(string); !ok || author == "" {
			t.Errorf("Expected non-empty author, got %v", values["header.author"])
		}
		if desc, ok := values["header.description"].(string); !ok || desc == "" {
			t.Errorf("Expected non-empty description, got %v", values["header.description"])
		}
	})

	t.Run("Metrics Values", func(t *testing.T) {
		if wingArea, ok := values["metrics.wing_area"].(float64); !ok || wingArea != 235.0 {
			t.Errorf("Expected wing area 235.0, got %v", values["metrics.wing_area"])
		}
		if wingSpan, ok := values["metrics.wing_span"].(float64); !ok || wingSpan != 37.1 {
			t.Errorf("Expected wing span 37.1, got %v", values["metrics.wing_span"])
		}
	})

	t.Run("Mass Balance Values", func(t *testing.T) {
		if emptyMass, ok := values["mass_balance.empty_mass"].(float64); !ok || emptyMass != 7125.0 {
			t.Errorf("Expected empty mass 7125.0, got %v", values["mass_balance.empty_mass"])
		}
		if ixx, ok := values["mass_balance.ixx"].(float64); !ok || ixx != 8031.0 {
			t.Errorf("Expected Ixx 8031.0, got %v", values["mass_balance.ixx"])
		}
	})

	t.Run("Ground Reactions", func(t *testing.T) {
		// Test that we have contact points
		leftMLG, ok := values["ground_reactions.contact.0"].(map[string]interface{})
		if !ok {
			t.Errorf("Expected left MLG contact to be present")
			return
		}
		if name, ok := leftMLG["name"].(string); !ok || name != "LEFT_MLG" {
			t.Errorf("Expected left MLG name 'LEFT_MLG', got %v", leftMLG["name"])
		}
	})

	t.Run("Propulsion", func(t *testing.T) {
		// Test engine
		engine, ok := values["propulsion.engine.0"].(map[string]interface{})
		if !ok {
			t.Errorf("Expected engine to be present")
			return
		}
		if file, ok := engine["file"].(string); !ok || file != "Packard-V-1650-7" {
			t.Errorf("Expected engine file 'Packard-V-1650-7', got %v", engine["file"])
		}

		// Test tanks
		tank0, ok := values["propulsion.tank.0"].(map[string]interface{})
		if !ok {
			t.Errorf("Expected tank 0 to be present")
			return
		}
		if tankType, ok := tank0["type"].(string); !ok || tankType != "FUEL" {
			t.Errorf("Expected tank type 'FUEL', got %v", tank0["type"])
		}
	})

	t.Run("Flight Control", func(t *testing.T) {
		if fcName, ok := values["flight_control.name"].(string); !ok || fcName != "FCS: P51D" {
			t.Errorf("Expected flight control name 'FCS: P51D', got %v", values["flight_control.name"])
		}
	})

	t.Run("Aerodynamics", func(t *testing.T) {
		// Test that we have some aerodynamics data
		found := false
		for key := range values {
			if strings.Contains(key, "aerodynamics") {
				found = true
				break
			}
		}
		if !found {
			t.Errorf("Expected some aerodynamics data to be present")
		}
	})

	// Log the total number of extracted values for verification
	t.Logf("Total values extracted from real P-51D file: %d", len(values))

	// Ensure we're extracting a substantial amount of data
	if len(values) < 50 {
		t.Errorf("Expected at least 50 extracted values from real file, got %d", len(values))
	}

	// Test that we have data from all major sections
	expectedSections := []string{
		"header.author",
		"metrics.wing_area",
		"mass_balance.ixx", 
		"ground_reactions.contact.0",
		"propulsion.engine.0",
		"flight_control.name",
	}

	for _, section := range expectedSections {
		if _, exists := values[section]; !exists {
			t.Errorf("Expected section '%s' to be present in real file extraction", section)
		}
	}
}
