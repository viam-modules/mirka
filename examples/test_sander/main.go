package main

// Usage:
//   go run ./examples/test_sander status
//   go run ./examples/test_sander start --rpm 4000
//   go run ./examples/test_sander stop
//   go run ./examples/test_sander set-speed 6000
//   go run ./examples/test_sander monitor --interval 1
//   go run ./examples/test_sander bench-test
//
// Environment variables:
//   VIAM_ADDRESS           (required, machine remote address)
//   MIRKA_COMPONENT        (optional, default: mirka-sander)
//
// Auth comes from the Viam CLI's cached session — run `viam login` first.

import (
	"bufio"
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
	"time"

	"go.viam.com/rdk/cli"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
)

func getenv(name string) string {
	value, ok := os.LookupEnv(name)
	if !ok {
		log.Fatalf("missing env var %s", name)
	}
	return value
}

func connect(ctx context.Context) (robot.Robot, error) {
	logger := logging.NewLogger("test-sander")
	return cli.ConnectToMachine(ctx, getenv("VIAM_ADDRESS"), logger)
}

func fmtStatus(s map[string]interface{}) string {
	// Extract fields with safe type assertions
	operationState := getString(s, "operation_state")
	running := getString(s, "running")
	speedSetpoint := getString(s, "speed_setpoint_rpm")
	averageSpeed := getString(s, "average_speed_rpm")
	toolTemp := getString(s, "tool_temp_c")
	driveTemp := getString(s, "drive_temp_c")

	alarmsStr := "—"
	if alarms, ok := s["alarm_flags"].([]interface{}); ok && len(alarms) > 0 {
		var alarmStrs []string
		for _, a := range alarms {
			alarmStrs = append(alarmStrs, fmt.Sprintf("%v", a))
		}
		alarmsStr = strings.Join(alarmStrs, ", ")
	}

	return fmt.Sprintf("state=%-7s running=%-5s setpoint=%s rpm  actual=%s rpm  tool=%s°C  drive=%s°C  alarms=[%s]",
		operationState, running, speedSetpoint, averageSpeed, toolTemp, driveTemp, alarmsStr)
}

func getString(m map[string]interface{}, key string) string {
	if v, ok := m[key]; ok {
		return fmt.Sprintf("%v", v)
	}
	return ""
}

func cmdStatus(ctx context.Context, sander resource.Resource) error {
	result, err := sander.DoCommand(ctx, map[string]interface{}{"command": "status"})
	if err != nil {
		return err
	}
	fmt.Println(fmtStatus(result))
	return nil
}

func cmdStart(ctx context.Context, sander resource.Resource, rpm int) error {
	result, err := sander.DoCommand(ctx, map[string]interface{}{"command": "start", "rpm": rpm})
	if err != nil {
		return err
	}
	fmt.Println(result)
	return nil
}

func cmdStop(ctx context.Context, sander resource.Resource) error {
	result, err := sander.DoCommand(ctx, map[string]interface{}{"command": "stop"})
	if err != nil {
		return err
	}
	fmt.Println(result)
	return nil
}

func cmdSetSpeed(ctx context.Context, sander resource.Resource, rpm int) error {
	result, err := sander.DoCommand(ctx, map[string]interface{}{"command": "set_speed", "rpm": rpm})
	if err != nil {
		return err
	}
	fmt.Println(result)
	return nil
}

func cmdMonitor(ctx context.Context, sander resource.Resource, interval float64) error {
	fmt.Printf("polling every %vs — Ctrl-C to stop\n", interval)
	ticker := time.NewTicker(time.Duration(interval*1000) * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			fmt.Println("stopped")
			return nil
		case <-ticker.C:
			result, err := sander.DoCommand(ctx, map[string]interface{}{"command": "status"})
			if err != nil {
				return err
			}
			fmt.Printf("%s  %s\n", time.Now().Format("15:04:05"), fmtStatus(result))
		}
	}
}

// Canonical first-spin verification: clamped sander, no disc, low RPM, short run.
// Prompts before any motion to avoid surprises.
func cmdBenchTest(ctx context.Context, sander resource.Resource) error {
	fmt.Println("BENCH TEST — pre-flight checks:")
	fmt.Println("  1. Sander is clamped to a fixture and cannot move")
	fmt.Println("  2. Sanding disc is REMOVED")
	fmt.Println("  3. No body parts or loose items within arm's reach")
	fmt.Println("  4. Cabinet E-stop reachable")

	reader := bufio.NewReader(os.Stdin)
	fmt.Print("All four confirmed? (type YES to continue): ")
	resp, _ := reader.ReadString('\n')
	resp = strings.TrimSpace(resp)
	if resp != "YES" {
		fmt.Println("aborted")
		return nil
	}

	fmt.Println("\n[1/4] status before start")
	result, err := sander.DoCommand(ctx, map[string]interface{}{"command": "status"})
	if err != nil {
		return err
	}
	fmt.Printf("  %s\n", fmtStatus(result))

	fmt.Println("\n[2/4] start at 4000 rpm")
	_, err = sander.DoCommand(ctx, map[string]interface{}{"command": "start", "rpm": 4000})
	if err != nil {
		return err
	}
	time.Sleep(500 * time.Millisecond)
	result, err = sander.DoCommand(ctx, map[string]interface{}{"command": "status"})
	if err != nil {
		return err
	}
	fmt.Printf("  %s\n", fmtStatus(result))

	fmt.Println("\n[3/4] hold 2s")
	time.Sleep(2 * time.Second)
	result, err = sander.DoCommand(ctx, map[string]interface{}{"command": "status"})
	if err != nil {
		return err
	}
	fmt.Printf("  %s\n", fmtStatus(result))

	fmt.Println("\n[4/4] stop")
	_, err = sander.DoCommand(ctx, map[string]interface{}{"command": "stop"})
	if err != nil {
		return err
	}
	time.Sleep(500 * time.Millisecond)
	result, err = sander.DoCommand(ctx, map[string]interface{}{"command": "status"})
	if err != nil {
		return err
	}
	fmt.Printf("  %s\n", fmtStatus(result))

	fmt.Println("\ndone")
	return nil
}

func main() {
	componentFlag := flag.String(
		"component",
		func() string {
			if c := os.Getenv("MIRKA_COMPONENT"); c != "" {
				return c
			}
			return "mirka-sander"
		}(),
		"resource name of the generic component (default: mirka-sander or $MIRKA_COMPONENT)",
	)

	flag.Parse()

	args := flag.Args()
	if len(args) == 0 {
		log.Fatal("missing subcommand: status, start, stop, set-speed, monitor, or bench-test")
	}

	cmd := args[0]
	ctx := context.Background()

	machine, err := connect(ctx)
	if err != nil {
		log.Fatalf("failed to connect: %v", err)
	}
	defer machine.Close(ctx)

	sander, err := generic.FromRobot(machine, *componentFlag)
	if err != nil {
		log.Fatalf("failed to get component: %v", err)
	}

	switch cmd {
	case "status":
		if err := cmdStatus(ctx, sander); err != nil {
			log.Fatalf("status failed: %v", err)
		}

	case "start":
		rpm := 4000
		fs := flag.NewFlagSet("start", flag.ExitOnError)
		fs.IntVar(&rpm, "rpm", 4000, "RPM (default: 4000)")
		fs.Parse(args[1:])
		if err := cmdStart(ctx, sander, rpm); err != nil {
			log.Fatalf("start failed: %v", err)
		}

	case "stop":
		if err := cmdStop(ctx, sander); err != nil {
			log.Fatalf("stop failed: %v", err)
		}

	case "set-speed":
		if len(args) < 2 {
			log.Fatal("set-speed requires RPM argument")
		}
		rpm, err := strconv.Atoi(args[1])
		if err != nil {
			log.Fatalf("invalid RPM: %v", err)
		}
		if err := cmdSetSpeed(ctx, sander, rpm); err != nil {
			log.Fatalf("set-speed failed: %v", err)
		}

	case "monitor":
		interval := 1.0
		fs := flag.NewFlagSet("monitor", flag.ExitOnError)
		fs.Float64Var(&interval, "interval", 1.0, "polling interval in seconds (default: 1.0)")
		fs.Parse(args[1:])

		if err := cmdMonitor(ctx, sander, interval); err != nil {
			log.Fatalf("monitor failed: %v", err)
		}

	case "bench-test":
		if err := cmdBenchTest(ctx, sander); err != nil {
			log.Fatalf("bench-test failed: %v", err)
		}

	default:
		log.Fatalf("unknown command: %s", cmd)
	}
}
