package main

// Usage:
//   go run ./examples/test_remover status
//   go run ./examples/test_remover home
//   go run ./examples/test_remover set-position 1 --mm 12.5
//   go run ./examples/test_remover blow --seconds 0.5
//   go run ./examples/test_remover release-disc
//   go run ./examples/test_remover quit-error
//   go run ./examples/test_remover bench-test
//
// Environment variables:
//   VIAM_ADDRESS             (required)
//   VIAM_API_KEY_ID          (required)
//   VIAM_API_KEY             (required)
//   MIRKA_REMOVER_COMPONENT  (optional, default: mirka-remover)

import (
	"bufio"
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"

	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

func getenv(name string) string {
	value, ok := os.LookupEnv(name)
	if !ok {
		log.Fatalf("missing env var %s", name)
	}
	return value
}

func connect(ctx context.Context) (*client.RobotClient, error) {
	// Create API key credentials with proper JSON payload format
	keyData := map[string]string{
		"key_id": getenv("VIAM_API_KEY_ID"),
		"key":    getenv("VIAM_API_KEY"),
	}
	payload, err := json.Marshal(keyData)
	if err != nil {
		return nil, fmt.Errorf("failed to marshal credentials: %w", err)
	}

	credentials := rpc.Credentials{
		Type:    rpc.CredentialsTypeAPIKey,
		Payload: string(payload),
	}

	logger := logging.NewLogger("test-remover")
	dialOpts := []client.DialOption{
		client.WithDialDebug(),
		client.WithCredentials(credentials),
	}
	opts := []client.RobotClientOption{
		client.WithDialOptions(dialOpts...),
	}
	return client.New(ctx, getenv("VIAM_ADDRESS"), logger, opts...)
}

func boolLetter(v interface{}) string {
	if b, ok := v.(bool); ok && b {
		return "T"
	}
	return "F"
}

func fmtStatus(s map[string]interface{}) string {
	posMM, _ := s["position_mm"].(float64)
	return fmt.Sprintf("pos=%v homed=%s in=%s out=%s moving=%s err=%s actual=%.2fmm",
		s["position"],
		boolLetter(s["homed"]),
		boolLetter(s["state_in"]),
		boolLetter(s["state_out"]),
		boolLetter(s["moving"]),
		boolLetter(s["device_error"]),
		posMM,
	)
}

// confirm prints prompt and reports whether the operator typed exactly "YES".
func confirm(reader *bufio.Reader, prompt string) bool {
	fmt.Print(prompt)
	resp, _ := reader.ReadString('\n')
	return strings.TrimSpace(resp) == "YES"
}

func cmdStatus(ctx context.Context, remover resource.Resource) error {
	result, err := remover.DoCommand(ctx, map[string]interface{}{"command": "status"})
	if err != nil {
		return err
	}
	fmt.Println(fmtStatus(result))
	return nil
}

// Homing drives the plate to its reference position; it must be empty first
// or the manual's stated failure mode is a collision (manual p.49).
func cmdHome(ctx context.Context, remover resource.Resource) error {
	reader := bufio.NewReader(os.Stdin)
	if !confirm(reader, "HOMING: the sliding plate MUST be removed first (manual p.49). Type YES to continue: ") {
		fmt.Println("aborted")
		return nil
	}
	result, err := remover.DoCommand(ctx, map[string]interface{}{"command": "home"})
	if err != nil {
		return err
	}
	fmt.Println(result)
	return nil
}

func cmdSetPosition(ctx context.Context, remover resource.Resource, position int, mm float64) error {
	payload := map[string]interface{}{"command": "set_position", "position": position}
	if mm >= 0 {
		payload["mm"] = mm
	}
	result, err := remover.DoCommand(ctx, payload)
	if err != nil {
		return err
	}
	fmt.Println(result)
	return nil
}

func cmdBlow(ctx context.Context, remover resource.Resource, seconds float64) error {
	payload := map[string]interface{}{"command": "blow"}
	if seconds >= 0 {
		payload["seconds"] = seconds
	}
	result, err := remover.DoCommand(ctx, payload)
	if err != nil {
		return err
	}
	fmt.Println(result)
	return nil
}

func cmdReleaseDisc(ctx context.Context, remover resource.Resource) error {
	result, err := remover.DoCommand(ctx, map[string]interface{}{"command": "release_disc"})
	if err != nil {
		return err
	}
	fmt.Println(result)
	return nil
}

func cmdQuitError(ctx context.Context, remover resource.Resource) error {
	result, err := remover.DoCommand(ctx, map[string]interface{}{"command": "quit_error"})
	if err != nil {
		return err
	}
	fmt.Println(result)
	return nil
}

// cmdBenchTest runs the autochanger through every reachable position plus a
// blow cycle, printing status after each step so drift from the geometry
// constants shows up immediately rather than on the first production swap.
func cmdBenchTest(ctx context.Context, remover resource.Resource) error {
	reader := bufio.NewReader(os.Stdin)

	printStatus := func() error {
		result, err := remover.DoCommand(ctx, map[string]interface{}{"command": "status"})
		if err != nil {
			return err
		}
		fmt.Printf("  %s\n", fmtStatus(result))
		return nil
	}

	fmt.Println("\n[1/7] status")
	if err := printStatus(); err != nil {
		return err
	}

	fmt.Println("\n[2/7] home")
	fmt.Println("Verify envelope geometry constants in autochanger/geometry.go against the physical unit while you're at the cell.")
	if !confirm(reader, "HOMING: the sliding plate MUST be removed first (manual p.49). Type YES to continue: ") {
		fmt.Println("aborted")
		return nil
	}
	if _, err := remover.DoCommand(ctx, map[string]interface{}{"command": "home"}); err != nil {
		return err
	}
	if err := printStatus(); err != nil {
		return err
	}

	steps := []struct {
		label   string
		payload map[string]interface{}
	}{
		{"[3/7] set-position 1", map[string]interface{}{"command": "set_position", "position": 1}},
		{"[4/7] set-position 2", map[string]interface{}{"command": "set_position", "position": 2}},
		{"[5/7] set-position 3", map[string]interface{}{"command": "set_position", "position": 3}},
		{"[6/7] blow 0.5s", map[string]interface{}{"command": "blow", "seconds": 0.5}},
		{"[7/7] set-position 1", map[string]interface{}{"command": "set_position", "position": 1}},
	}
	for _, step := range steps {
		fmt.Printf("\n%s\n", step.label)
		if _, err := remover.DoCommand(ctx, step.payload); err != nil {
			return err
		}
		if err := printStatus(); err != nil {
			return err
		}
	}

	fmt.Println("\ndone")
	return nil
}

func main() {
	componentFlag := flag.String(
		"component",
		func() string {
			if c := os.Getenv("MIRKA_REMOVER_COMPONENT"); c != "" {
				return c
			}
			return "mirka-remover"
		}(),
		"resource name of the generic component (default: mirka-remover or $MIRKA_REMOVER_COMPONENT)",
	)

	flag.Parse()

	args := flag.Args()
	if len(args) == 0 {
		log.Fatal("missing subcommand: status, home, set-position, blow, release-disc, quit-error, or bench-test")
	}

	cmd := args[0]
	ctx := context.Background()

	machine, err := connect(ctx)
	if err != nil {
		log.Fatalf("failed to connect: %v", err)
	}
	defer machine.Close(ctx)

	remover, err := generic.FromRobot(machine, *componentFlag)
	if err != nil {
		log.Fatalf("failed to get component: %v", err)
	}

	switch cmd {
	case "status":
		if err := cmdStatus(ctx, remover); err != nil {
			log.Fatalf("status failed: %v", err)
		}

	case "home":
		if err := cmdHome(ctx, remover); err != nil {
			log.Fatalf("home failed: %v", err)
		}

	case "set-position":
		if len(args) < 2 {
			log.Fatal("set-position requires POSITION argument (1, 2, or 3)")
		}
		position, err := strconv.Atoi(args[1])
		if err != nil {
			log.Fatalf("invalid position: %v", err)
		}
		mm := -1.0
		fs := flag.NewFlagSet("set-position", flag.ExitOnError)
		fs.Float64Var(&mm, "mm", -1.0, "target position in mm (optional)")
		fs.Parse(args[2:])
		if err := cmdSetPosition(ctx, remover, position, mm); err != nil {
			log.Fatalf("set-position failed: %v", err)
		}

	case "blow":
		seconds := -1.0
		fs := flag.NewFlagSet("blow", flag.ExitOnError)
		fs.Float64Var(&seconds, "seconds", -1.0, "blow duration in seconds (optional)")
		fs.Parse(args[1:])
		if err := cmdBlow(ctx, remover, seconds); err != nil {
			log.Fatalf("blow failed: %v", err)
		}

	case "release-disc":
		if err := cmdReleaseDisc(ctx, remover); err != nil {
			log.Fatalf("release-disc failed: %v", err)
		}

	case "quit-error":
		if err := cmdQuitError(ctx, remover); err != nil {
			log.Fatalf("quit-error failed: %v", err)
		}

	case "bench-test":
		if err := cmdBenchTest(ctx, remover); err != nil {
			log.Fatalf("bench-test failed: %v", err)
		}

	default:
		log.Fatalf("unknown command: %s", cmd)
	}
}
