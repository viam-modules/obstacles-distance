package main

import (
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"

	obstaclesdistance "obstaclesdistance/obstacles-distance"
)

func main() {
	module.ModularMain(resource.APIModel{
		API:   vision.API,
		Model: obstaclesdistance.Model,
	})
}
