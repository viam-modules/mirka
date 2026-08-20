package main

import (
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"

	"github.com/viam-modules/mirka/airos"
	"github.com/viam-modules/mirka/autochanger"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: generic.API, Model: airos.Model},
		resource.APIModel{API: generic.API, Model: autochanger.RemoverModel},
	)
}
