GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/viam-mirka

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	GO_BUILD_FLAGS := -tags no_cgo
	MODULE_BINARY = bin/viam-mirka.exe
endif

# The generated template lists `*.go cmd/module/*.go`, which suits a module whose
# code sits at the repo root. This one has packages in subdirectories, so the
# prerequisites are found rather than enumerated -- an enumerated list silently
# serves a stale binary the first time someone adds a package and forgets it.
$(MODULE_BINARY): Makefile go.mod $(shell find . -name '*.go' -not -path './bin/*')
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) ./cmd/module

lint:
	gofmt -s -w .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

test:
	go test ./...

FIRST_RUN := $(shell jq -r '.first_run // empty' meta.json 2>/dev/null)
TAR_FILES := meta.json $(MODULE_BINARY)
ifneq ($(FIRST_RUN),)
TAR_FILES += $(FIRST_RUN)
endif

module.tar.gz: meta.json $(MODULE_BINARY)
ifneq ($(VIAM_TARGET_OS), windows)
	strip $(MODULE_BINARY)
endif
	tar czf $@ $(TAR_FILES)

module: test module.tar.gz

all: test module.tar.gz

setup:
	go mod tidy
