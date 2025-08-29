package main

import (
	"encoding/json"
	"fmt"
)


func PrettyPrint(v map[string]interface{}) {
        data, err := json.MarshalIndent(v, "", "  ")
        if err != nil {
                fmt.Println("error:", err)
                return
        }
        fmt.Println(string(data))
}