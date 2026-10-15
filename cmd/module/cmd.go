package main

import (
	"go.viam.com/rdk/components/gantry"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"

	"github.com/viam-modules/mirka/airos"
	"github.com/viam-modules/mirka/autochanger"
	"github.com/viam-modules/mirka/removersvc"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: generic.API, Model: airos.Model},
		resource.APIModel{API: gantry.API, Model: autochanger.Model},
		resource.APIModel{API: genericservice.API, Model: removersvc.Model},
	)
}
