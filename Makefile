viam-mirka: airos/*.go cmd/module/*.go go.mod go.sum
	GOOS=linux GOARCH=arm64 go build -o viam-mirka cmd/module/cmd.go

test:
	go test ./...

lint:
	gofmt -w -s .

module.tar.gz: viam-mirka
	tar czf module.tar.gz meta.json viam-mirka

module: module.tar.gz

all: module test

update:
	go get go.viam.com/rdk@latest
	go mod tidy
