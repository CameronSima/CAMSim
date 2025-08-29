package main

import (
	"encoding/json"
	"fmt"
	"os"
)

func prettyPrint(v map[string]interface{}) {
        data, err := json.MarshalIndent(v, "", "  ")
        if err != nil {
                fmt.Println("error:", err)
                return
        }
        fmt.Println(string(data))
}

func main() {
    // Open JSBSim XML file
    file, err := os.Open("aircraft/p51d-jsbsim.xml")
    if err != nil {
        panic(err)
    }
    defer file.Close()
    
    // Parse the configuration
    config, err := ParseJSBSimConfig(file)
    if err != nil {
        panic(err)
    }
    
    // Extract all values
    values := ExtractAllValues(config)
    
    // Access specific values
    if wingArea, ok := values["metrics.wing_area"]; ok {
        fmt.Printf("Wing Area: %v sq ft\n", wingArea)
    }

	//prettyPrint(values)
    
    // Example of table interpolation
    if config.Aerodynamics != nil {
        fmt.Printf("Found aerodynamics section\n")
        fmt.Printf("Number of axes: %d\n", len(config.Aerodynamics.Axis))
        fmt.Printf("Number of standalone functions: %d\n", len(config.Aerodynamics.Function))
        
        // Check standalone functions first
        for i, fn := range config.Aerodynamics.Function {
            fmt.Printf("Standalone function %d: %s\n", i, fn.Name)
            if fn.Table != nil {
                fmt.Printf("  Found table in standalone function: %s\n", fn.Table.Name)
                pt, err := ParseTable(fn.Table)
                if err != nil {
                    fmt.Printf("  Error parsing table: %v\n", err)
                    continue
                }
                // Interpolate at specific conditions - use single input for 1D table
                result, err := InterpolateTable(pt, 0.1) // single input for 1D table
                if err != nil {
                    fmt.Printf("  Error interpolating: %v\n", err)
                } else {
                    fmt.Printf("  Interpolated value: %f\n", result)
                }
            }
        }
        
        // Check axis functions
        for _, axis := range config.Aerodynamics.Axis {
            fmt.Printf("Axis: %s, Functions: %d\n", axis.Name, len(axis.Function))
            for j, fn := range axis.Function {
                fmt.Printf("  Function %d: %s\n", j, fn.Name)
                
                // Check for direct table
                if fn.Table != nil {
                    fmt.Printf("    Found direct table in axis function: %s\n", fn.Table.Name)
                    pt, err := ParseTable(fn.Table)
                    if err != nil {
                        fmt.Printf("    Error parsing table: %v\n", err)
                        continue
                    }
                    // Try to determine table dimension and interpolate accordingly
                    if pt.Dimension == 1 {
                        result, err := InterpolateTable(pt, 0.1) // single input for 1D
                        if err != nil {
                            fmt.Printf("    Error interpolating 1D: %v\n", err)
                        } else {
                            fmt.Printf("    Interpolated 1D value: %f\n", result)
                        }
                    } else if pt.Dimension == 2 {
                        result, err := InterpolateTable(pt, 0.1, 10.0) // two inputs for 2D
                        if err != nil {
                            fmt.Printf("    Error interpolating 2D: %v\n", err)
                        } else {
                            fmt.Printf("    Interpolated 2D value: %f\n", result)
                        }
                    }
                }
                
                // Check for table inside operations
                if fn.Product != nil && fn.Product.Table != nil {
                    fmt.Printf("    Found table in Product operation: %s\n", fn.Product.Table.Name)
                    pt, err := ParseTable(fn.Product.Table)
                    if err != nil {
                        fmt.Printf("    Error parsing product table: %v\n", err)
                        continue
                    }
                    if pt.Dimension == 1 {
                        result, err := InterpolateTable(pt, 0.1) // single input for 1D
                        if err != nil {
                            fmt.Printf("    Error interpolating product 1D: %v\n", err)
                        } else {
                            fmt.Printf("    Interpolated product 1D value: %f\n", result)
                        }
                    }
                }
                
                // Could also check other operations like Sum, Difference, etc. if needed
            }
        }
    } else {
        fmt.Printf("No aerodynamics section found\n")
    }
}