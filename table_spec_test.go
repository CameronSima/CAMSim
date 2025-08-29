package main

import (
	"fmt"
	"strings"
	"testing"
	"time"
)

// TestJSBSimTableSpecification tests our table parsing against the exact JSBSim specification
func TestJSBSimTableSpecification(t *testing.T) {
	
	t.Run("1D Table Specification", func(t *testing.T) {
		// Create a 1D table following JSBSim specification format:
		// <independentVar>property_name</independentVar>
		// <tableData>row_index_1 data_1 ...</tableData>
		table := &Table{
			Name: "test_1d_spec",
			IndependentVar: []*IndependentVar{
				{Value: "aero/alpha-deg"},
			},
			TableData: []*TableData{
				{Data: "-10.0  0.1\n0.0    0.0\n5.0    0.05\n10.0   0.1"},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse 1D table: %v", err)
		}

		// Verify structure matches specification
		assertEqual(t, pt.Dimension, 1)
		assertEqual(t, len(pt.IndependentVars), 1)
		assertEqual(t, pt.IndependentVars[0], "aero/alpha-deg")
		assertEqual(t, len(pt.LookupTypes), 1)
		assertEqual(t, pt.LookupTypes[0], "") // No lookup type for 1D

		// Verify data parsing
		if pt.Data1D == nil {
			t.Fatal("1D data should not be nil")
		}
		assertEqual(t, len(pt.Data1D.Indices), 4)
		assertEqual(t, len(pt.Data1D.Values), 4)
		assertEqual(t, pt.Data1D.Indices[0], -10.0)
		assertEqual(t, pt.Data1D.Values[1], 0.0)
		
		// Test interpolation
		result, err := InterpolateTable(pt, 2.5) // Between 0 and 5
		if err != nil {
			t.Fatalf("Failed to interpolate 1D: %v", err)
		}
		assertEqual(t, result, 0.025) // Should be halfway between 0.0 and 0.05
	})

	t.Run("2D Table Specification", func(t *testing.T) {
		// Example exactly as specified in JSBSim documentation
		table := &Table{
			Name: "test_2d_spec",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
				{Lookup: "column", Value: "velocities/mach"},
			},
			TableData: []*TableData{
				{Data: `        0.5    0.8    1.0
-10.0   0.1    0.2    0.3
0.0     0.0    0.1    0.2
10.0    0.1    0.3    0.5`},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse 2D table: %v", err)
		}

		// Verify structure matches specification
		assertEqual(t, pt.Dimension, 2)
		assertEqual(t, len(pt.IndependentVars), 2)
		assertEqual(t, pt.IndependentVars[0], "aero/alpha-deg")
		assertEqual(t, pt.IndependentVars[1], "velocities/mach")
		assertEqual(t, pt.LookupTypes[0], "row")
		assertEqual(t, pt.LookupTypes[1], "column")

		// Verify data structure
		if pt.Data2D == nil {
			t.Fatal("2D data should not be nil")
		}
		
		// Check column indices (first row)
		assertEqual(t, len(pt.Data2D.ColIndices), 3)
		assertEqual(t, pt.Data2D.ColIndices[0], 0.5)
		assertEqual(t, pt.Data2D.ColIndices[1], 0.8)
		assertEqual(t, pt.Data2D.ColIndices[2], 1.0)
		
		// Check row indices (first column)
		assertEqual(t, len(pt.Data2D.RowIndices), 3)
		assertEqual(t, pt.Data2D.RowIndices[0], -10.0)
		assertEqual(t, pt.Data2D.RowIndices[1], 0.0)
		assertEqual(t, pt.Data2D.RowIndices[2], 10.0)
		
		// Check data values
		assertEqual(t, len(pt.Data2D.Data), 3) // 3 rows
		assertEqual(t, len(pt.Data2D.Data[0]), 3) // 3 columns
		assertEqual(t, pt.Data2D.Data[0][0], 0.1) // data_11 (alpha=-10, mach=0.5)
		assertEqual(t, pt.Data2D.Data[1][1], 0.1) // data_22 (alpha=0, mach=0.8)
		assertEqual(t, pt.Data2D.Data[2][2], 0.5) // data_33 (alpha=10, mach=1.0)
		
		// Test 2D interpolation
		result, err := InterpolateTable(pt, 0.0, 0.65) // alpha=0, mach between 0.5 and 0.8
		if err != nil {
			t.Fatalf("Failed to interpolate 2D: %v", err)
		}
		assertEqual(t, result, 0.05) // Should interpolate between 0.0 and 0.1
	})

	t.Run("3D Table Specification", func(t *testing.T) {
		// Example exactly as specified in JSBSim documentation
		table := &Table{
			Name: "test_3d_spec",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
				{Lookup: "column", Value: "velocities/mach"},
				{Lookup: "table", Value: "atmosphere/altitude-ft"},
			},
			TableData: []*TableData{
				{
					Breakpoint: "0.0", // Sea level
					Data: `        0.5    1.0
-10.0   0.1    0.2
10.0    0.3    0.4`,
				},
				{
					Breakpoint: "10000.0", // 10,000 ft
					Data: `        0.5    1.0
-10.0   0.15   0.25
10.0    0.35   0.45`,
				},
				{
					Breakpoint: "20000.0", // 20,000 ft
					Data: `        0.5    1.0
-10.0   0.2    0.3
10.0    0.4    0.5`,
				},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse 3D table: %v", err)
		}

		// Verify structure matches specification
		assertEqual(t, pt.Dimension, 3)
		assertEqual(t, len(pt.IndependentVars), 3)
		assertEqual(t, pt.IndependentVars[0], "aero/alpha-deg")
		assertEqual(t, pt.IndependentVars[1], "velocities/mach")
		assertEqual(t, pt.IndependentVars[2], "atmosphere/altitude-ft")
		assertEqual(t, pt.LookupTypes[0], "row")
		assertEqual(t, pt.LookupTypes[1], "column")
		assertEqual(t, pt.LookupTypes[2], "table")

		// Verify 3D data structure
		if pt.Data3D == nil {
			t.Fatal("3D data should not be nil")
		}
		assertEqual(t, len(pt.Data3D), 3) // 3 altitude tables
		
		// Check first table (sea level)
		assertEqual(t, pt.Data3D[0].Breakpoint, 0.0)
		assertEqual(t, len(pt.Data3D[0].RowIndices), 2)
		assertEqual(t, len(pt.Data3D[0].ColIndices), 2)
		assertEqual(t, pt.Data3D[0].Data[0][0], 0.1)
		assertEqual(t, pt.Data3D[0].Data[1][1], 0.4)
		
		// Check second table (10,000 ft)
		assertEqual(t, pt.Data3D[1].Breakpoint, 10000.0)
		assertEqual(t, pt.Data3D[1].Data[0][0], 0.15)
		assertEqual(t, pt.Data3D[1].Data[1][1], 0.45)
		
		// Check third table (20,000 ft)
		assertEqual(t, pt.Data3D[2].Breakpoint, 20000.0)
		assertEqual(t, pt.Data3D[2].Data[0][0], 0.2)
		assertEqual(t, pt.Data3D[2].Data[1][1], 0.5)
		
		// Test 3D interpolation
		result, err := InterpolateTable(pt, 0.0, 0.75, 5000.0) // Mid-values
		if err != nil {
			t.Fatalf("Failed to interpolate 3D: %v", err)
		}
		
		// Let me manually calculate what this should be:
		// Alpha=0 (halfway between -10 and 10)
		// Mach=0.75 (halfway between 0.5 and 1.0) 
		// Alt=5000 (halfway between 0 and 10000)
		
		// At sea level (alt=0):
		//   alpha=-10, mach=0.5: 0.1, mach=1.0: 0.2 -> at mach=0.75: 0.15
		//   alpha=10,  mach=0.5: 0.3, mach=1.0: 0.4 -> at mach=0.75: 0.35
		//   at alpha=0: halfway between 0.15 and 0.35 = 0.25
		
		// At 10000ft (alt=10000):
		//   alpha=-10, mach=0.5: 0.15, mach=1.0: 0.25 -> at mach=0.75: 0.20
		//   alpha=10,  mach=0.5: 0.35, mach=1.0: 0.45 -> at mach=0.75: 0.40
		//   at alpha=0: halfway between 0.20 and 0.40 = 0.30
		
		// At alt=5000: halfway between 0.25 and 0.30 = 0.275
		
		t.Logf("3D interpolation result: %f", result)
		assertApproxEqual(t, result, 0.275, 0.01)
	})
}

// TestRealWorldJSBSimTables tests with actual table formats from real JSBSim files
func TestRealWorldJSBSimTables(t *testing.T) {
	
	t.Run("Real 1D Ground Effect Table", func(t *testing.T) {
		// Actual table from P-51D file
		table := &Table{
			Name: "kCLge",
			IndependentVar: []*IndependentVar{
				{Value: "aero/h_b-mac-ft"},
			},
			TableData: []*TableData{
				{Data: `0.0000  1.2290
0.1000  1.1240
0.1500  1.1160
0.2000  1.1240
0.3000  1.1050
0.4000  1.0410
0.5000  1.0340
0.6000  1.0190
0.7000  1.0080
0.8000  1.0030
0.9000  1.0010
1.0000  1.0000
1.1000  1.0000`},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse real 1D table: %v", err)
		}

		assertEqual(t, pt.Dimension, 1)
		assertEqual(t, len(pt.Data1D.Indices), 13)
		assertEqual(t, pt.Data1D.Indices[0], 0.0)
		assertEqual(t, pt.Data1D.Values[0], 1.229)
		
		// Test interpolation at ground level (should give max ground effect)
		result, err := InterpolateTable(pt, 0.0)
		if err != nil {
			t.Fatalf("Failed to interpolate: %v", err)
		}
		assertEqual(t, result, 1.229)
		
		// Test interpolation at high altitude (should give no ground effect)
		result, err = InterpolateTable(pt, 1.0)
		if err != nil {
			t.Fatalf("Failed to interpolate: %v", err)
		}
		assertEqual(t, result, 1.0)
	})
	
	t.Run("Real 2D CLalpha Table", func(t *testing.T) {
		// Simplified version of actual P-51D CLalpha table
		table := &Table{
			Name: "CLalpha",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
				{Lookup: "column", Value: "aero/Re"},
			},
			TableData: []*TableData{
				{Data: `           2500000   5000000   10000000
-10.0      -0.8      -0.9      -0.95
0.0         0.16      0.16      0.16
5.0         0.71      0.71      0.72
10.0        1.14      1.18      1.22
15.0        1.40      1.48      1.57`},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse real 2D table: %v", err)
		}

		assertEqual(t, pt.Dimension, 2)
		assertEqual(t, len(pt.Data2D.ColIndices), 3) // 3 Reynolds numbers
		assertEqual(t, len(pt.Data2D.RowIndices), 5) // 5 alpha values
		
		// Test interpolation at zero alpha, medium Reynolds number
		result, err := InterpolateTable(pt, 0.0, 7500000.0)
		if err != nil {
			t.Fatalf("Failed to interpolate: %v", err)
		}
		assertApproxEqual(t, result, 0.16, 0.001) // Should be 0.16 across all Re
		
		// Test lift curve slope (CL should increase with alpha)
		cl5, _ := InterpolateTable(pt, 5.0, 5000000.0)
		cl10, _ := InterpolateTable(pt, 10.0, 5000000.0)
		
		if cl10 <= cl5 {
			t.Errorf("Lift should increase with alpha: CL(5)=%f, CL(10)=%f", cl5, cl10)
		}
	})
	
	t.Run("Real 3D Brake Table", func(t *testing.T) {
		// Actual 3D brake table from P-51D file
		table := &Table{
			Name: "brake-scaling",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "velocities/vg-fps"},
				{Lookup: "column", Value: "gear/unit[2]/compression-ft"},
				{Lookup: "table", Value: "propulsion/engine/map-inhg"},
			},
			TableData: []*TableData{
				{
					Breakpoint: "11.0",
					Data: `        0.05   0.25
0      0.15    1.2
15     0.075   1.0
24     0.3     1.0
50     0.6     1.0`,
				},
				{
					Breakpoint: "20.0",
					Data: `        0.05    0.25
0      0.4     1.8
15     0.3     1.5
24     0.3     1.0
50     0.6     1.0`,
				},
				{
					Breakpoint: "30.0",
					Data: `        0.05    0.25
0      0.5     2.5
15     0.4     2.0
24     0.3     1.0
50     0.6     1.0`,
				},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse real 3D table: %v", err)
		}

		assertEqual(t, pt.Dimension, 3)
		assertEqual(t, len(pt.Data3D), 3) // 3 manifold pressure tables
		assertEqual(t, pt.Data3D[0].Breakpoint, 11.0)
		assertEqual(t, pt.Data3D[1].Breakpoint, 20.0)
		assertEqual(t, pt.Data3D[2].Breakpoint, 30.0)
		
		// Test that brake effectiveness increases with manifold pressure
		brake11, _ := InterpolateTable(pt, 0.0, 0.25, 11.0)
		brake30, _ := InterpolateTable(pt, 0.0, 0.25, 30.0)
		
		if brake30 <= brake11 {
			t.Errorf("Brake effectiveness should increase with MAP: MAP11=%f, MAP30=%f", brake11, brake30)
		}
	})
}

// TestTableParsingPerformance ensures our parsing is efficient
func TestTableParsingPerformance(t *testing.T) {
	if testing.Short() {
		t.Skip("Skipping performance test in short mode")
	}

	t.Run("Large 2D Table Performance", func(t *testing.T) {
		// Create a large 2D table (100x100)
		var data strings.Builder
		
		// Column headers (alpha values)
		for i := 0; i < 100; i++ {
			alpha := float64(i-50) * 0.5 // -25 to +25 degrees
			data.WriteString(fmt.Sprintf("%.1f ", alpha))
		}
		data.WriteString("\n")
		
		// Data rows (mach vs alpha matrix)
		for i := 0; i < 100; i++ {
			mach := float64(i) * 0.01 // 0.0 to 0.99 mach
			data.WriteString(fmt.Sprintf("%.2f ", mach))
			
			for j := 0; j < 100; j++ {
				alpha := float64(j-50) * 0.5
				// Simple mathematical relationship for testing
				cl := 0.1 * alpha * (1.0 + mach*0.1)
				data.WriteString(fmt.Sprintf("%.6f ", cl))
			}
			data.WriteString("\n")
		}

		table := &Table{
			Name: "large_performance_test",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "velocities/mach"},
				{Lookup: "column", Value: "aero/alpha-deg"},
			},
			TableData: []*TableData{
				{Data: data.String()},
			},
		}

		// Time the parsing
		start := time.Now()
		pt, err := ParseTable(table)
		parseTime := time.Since(start)

		if err != nil {
			t.Fatalf("Failed to parse large table: %v", err)
		}

		t.Logf("Parsed 100x100 table in %v", parseTime)
		
		// Should parse quickly (under 10ms for 10,000 data points)
		if parseTime > time.Millisecond*10 {
			t.Errorf("Table parsing too slow: %v", parseTime)
		}

		// Verify structure
		assertEqual(t, pt.Dimension, 2)
		assertEqual(t, len(pt.Data2D.ColIndices), 100)
		assertEqual(t, len(pt.Data2D.RowIndices), 100)
		assertEqual(t, len(pt.Data2D.Data), 100)

		// Test interpolation performance
		start = time.Now()
		for i := 0; i < 1000; i++ {
			mach := float64(i%50) * 0.01
			alpha := float64((i%50)-25) * 0.4
			_, err := InterpolateTable(pt, mach, alpha)
			if err != nil {
				t.Errorf("Interpolation failed: %v", err)
			}
		}
		interpTime := time.Since(start)
		
		t.Logf("1000 interpolations took %v (%.2f μs each)", interpTime, float64(interpTime.Nanoseconds())/1000000.0)
		
		// Should interpolate quickly (under 1ms for 1000 interpolations)
		if interpTime > time.Millisecond {
			t.Errorf("Interpolation too slow: %v", interpTime)
		}
	})
}

// TestTableDataFormats tests various data format edge cases
func TestTableDataFormats(t *testing.T) {
	
	t.Run("Mixed Whitespace Handling", func(t *testing.T) {
		table := &Table{
			Name: "whitespace_test",
			IndependentVar: []*IndependentVar{
				{Value: "test/prop"},
			},
			TableData: []*TableData{
				{Data: "  -10.0    0.1  \n\t0.0  \t  0.0\n    5.0\t0.05   \n\n10.0 \t 0.1  \n  "},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse table with mixed whitespace: %v", err)
		}

		assertEqual(t, len(pt.Data1D.Indices), 4)
		assertEqual(t, pt.Data1D.Indices[0], -10.0)
		assertEqual(t, pt.Data1D.Values[2], 0.05)
	})
	
	t.Run("Scientific Notation", func(t *testing.T) {
		table := &Table{
			Name: "scientific_test",
			IndependentVar: []*IndependentVar{
				{Value: "test/prop"},
			},
			TableData: []*TableData{
				{Data: "1.0e-3  2.5e-2\n1.5E+2  3.7E-1\n-2.1e+1 4.8e-4"},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse table with scientific notation: %v", err)
		}

		assertEqual(t, len(pt.Data1D.Indices), 3)
		assertEqual(t, pt.Data1D.Indices[0], 0.001)    // 1.0e-3
		assertEqual(t, pt.Data1D.Values[0], 0.025)     // 2.5e-2
		assertEqual(t, pt.Data1D.Indices[1], 150.0)    // 1.5E+2
		assertEqual(t, pt.Data1D.Values[1], 0.37)      // 3.7E-1
	})
	
	t.Run("Empty Lines and Comments", func(t *testing.T) {
		table := &Table{
			Name: "comments_test",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "alpha"},
				{Lookup: "column", Value: "mach"},
			},
			TableData: []*TableData{
				{Data: `
				
        0.5    0.8
				
-10.0   0.1    0.2

0.0     0.0    0.1

`},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse table with empty lines: %v", err)
		}

		assertEqual(t, pt.Dimension, 2)
		assertEqual(t, len(pt.Data2D.ColIndices), 2)
		assertEqual(t, len(pt.Data2D.RowIndices), 2)
	})
}
