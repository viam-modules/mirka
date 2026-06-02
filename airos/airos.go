package airos

import (
	"context"
	"errors"
	"fmt"
	"strings"
	"sync"
	"time"

	"github.com/simonvetter/modbus"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/utils"
)

var Model = resource.NewModel("viam", "mirka", "airos-550cv")

// Modbus register addresses are 0-indexed on the wire, but the Mirka manual
// documents them in 1-indexed block-prefixed form (40011, 30018, 00012, etc.).
// The constants below are the on-wire addresses; the trailing comment is the
// manual's reference for cross-checking against Mirka docs.
const (
	regSpeedSetpoint uint16 = 10 // 40011 holding
	regOperation     uint16 = 11 // 40012 holding
	regAvgSpeed      uint16 = 17 // 30018 input
	regToolTempC     uint16 = 18 // 30019 input
	regDriveTempC    uint16 = 19 // 30020 input
	regAlarmStatus   uint16 = 46 // 30047 input
	coilCabinetRelay uint16 = 11 // 00012 coil
)

const (
	opRun       uint16 = 0x0001
	opStop      uint16 = 0x0002
	opOn        uint16 = 0x0004
	opOff       uint16 = 0x0008
	opWPDisable uint16 = 0x0040
	opWPEnable  uint16 = 0x0080
)

const (
	minRPM uint16 = 4000
	maxRPM uint16 = 10000
)

type Config struct {
	SerialPath   string `json:"serial_path"`
	BaudRate     uint   `json:"baud_rate,omitempty"`
	Parity       string `json:"parity,omitempty"`
	SlaveAddress uint8  `json:"slave_address,omitempty"`
	DefaultRPM   uint16 `json:"default_rpm,omitempty"`
	TimeoutMs    uint   `json:"timeout_ms,omitempty"`
}

func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.SerialPath == "" {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "serial_path")
	}
	if c.DefaultRPM != 0 && (c.DefaultRPM < minRPM || c.DefaultRPM > maxRPM) {
		return nil, nil, fmt.Errorf("%s: default_rpm must be between %d and %d", path, minRPM, maxRPM)
	}
	switch c.Parity {
	case "", "E", "N", "O":
	default:
		return nil, nil, fmt.Errorf("%s: parity must be one of E, N, O", path)
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(generic.API, Model, resource.Registration[resource.Resource, *Config]{
		Constructor: newSander,
	})
}

func newSander(
	ctx context.Context,
	deps resource.Dependencies,
	config resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](config)
	if err != nil {
		return nil, err
	}

	baud := conf.BaudRate
	if baud == 0 {
		baud = 19200
	}
	parity := conf.Parity
	if parity == "" {
		parity = "E"
	}
	slave := conf.SlaveAddress
	if slave == 0 {
		slave = 86
	}
	defaultRPM := conf.DefaultRPM
	if defaultRPM == 0 {
		defaultRPM = 8000
	}
	timeoutMs := conf.TimeoutMs
	if timeoutMs == 0 {
		timeoutMs = 1000
	}

	client, err := modbus.NewClient(&modbus.ClientConfiguration{
		URL:      "rtu://" + conf.SerialPath,
		Speed:    baud,
		DataBits: 8,
		Parity:   parityFromString(parity),
		StopBits: 1,
		Timeout:  time.Duration(timeoutMs) * time.Millisecond,
	})
	if err != nil {
		return nil, fmt.Errorf("constructing modbus client: %w", err)
	}
	if err := client.SetUnitId(slave); err != nil {
		return nil, fmt.Errorf("setting modbus slave id: %w", err)
	}
	if err := client.Open(); err != nil {
		return nil, fmt.Errorf("opening serial port %s: %w", conf.SerialPath, err)
	}

	return &sander{
		name:       config.ResourceName(),
		client:     client,
		defaultRPM: defaultRPM,
		logger:     logger,
	}, nil
}

func parityFromString(p string) uint {
	switch p {
	case "E":
		return modbus.PARITY_EVEN
	case "O":
		return modbus.PARITY_ODD
	default:
		return modbus.PARITY_NONE
	}
}

type sander struct {
	resource.AlwaysRebuild

	name       resource.Name
	client     *modbus.ModbusClient
	defaultRPM uint16
	logger     logging.Logger

	mu      sync.Mutex
	running bool
}

func (s *sander) Name() resource.Name {
	return s.name
}

func (s *sander) start(rpm uint16) error {
	if rpm < minRPM || rpm > maxRPM {
		return fmt.Errorf("rpm %d out of range [%d, %d]", rpm, minRPM, maxRPM)
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	if err := s.client.WriteRegister(regOperation, opWPDisable); err != nil {
		return fmt.Errorf("disable write protection: %w", err)
	}
	if err := s.client.WriteRegister(regOperation, opOn); err != nil {
		return fmt.Errorf("write ON: %w", err)
	}
	if err := s.client.WriteRegister(regSpeedSetpoint, rpm); err != nil {
		return fmt.Errorf("write speed setpoint: %w", err)
	}
	if err := s.client.WriteRegister(regOperation, opRun); err != nil {
		return fmt.Errorf("write RUN: %w", err)
	}
	s.running = true
	return nil
}

func (s *sander) stop() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.stopLocked()
}

func (s *sander) stopLocked() error {
	if err := s.client.WriteRegister(regOperation, opWPDisable); err != nil {
		return fmt.Errorf("disable write protection: %w", err)
	}
	if err := s.client.WriteRegister(regOperation, opStop); err != nil {
		return fmt.Errorf("write STOP: %w", err)
	}
	if err := s.client.WriteRegister(regOperation, opOff); err != nil {
		return fmt.Errorf("write OFF: %w", err)
	}
	s.running = false
	return nil
}

func (s *sander) setSpeed(rpm uint16) error {
	if rpm < minRPM || rpm > maxRPM {
		return fmt.Errorf("rpm %d out of range [%d, %d]", rpm, minRPM, maxRPM)
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.client.WriteRegister(regOperation, opWPDisable); err != nil {
		return fmt.Errorf("disable write protection: %w", err)
	}
	return s.client.WriteRegister(regSpeedSetpoint, rpm)
}

func (s *sander) Status(ctx context.Context) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	inputs, err := s.client.ReadRegisters(regAvgSpeed, 3, modbus.INPUT_REGISTER)
	if err != nil {
		return nil, fmt.Errorf("read input registers: %w", err)
	}
	alarm, err := s.client.ReadRegisters(regAlarmStatus, 1, modbus.INPUT_REGISTER)
	if err != nil {
		return nil, fmt.Errorf("read alarm status: %w", err)
	}
	holding, err := s.client.ReadRegisters(regSpeedSetpoint, 2, modbus.HOLDING_REGISTER)
	if err != nil {
		return nil, fmt.Errorf("read holding registers: %w", err)
	}

	return map[string]interface{}{
		"running":            s.running,
		"average_speed_rpm":  int(inputs[0]),
		"tool_temp_c":        int(inputs[1]),
		"drive_temp_c":       int(inputs[2]),
		"speed_setpoint_rpm": int(holding[0]),
		"operation_state":    operationStateName(holding[1]),
		"alarm_status":       int(alarm[0]),
		"alarm_flags":        decodeAlarmFlags(alarm[0]),
	}, nil
}

var opStateBits = []struct {
	bit  uint16
	name string
}{
	{opRun, "RUN"},
	{opStop, "STOP"},
	{opOn, "ON"},
	{opOff, "OFF"},
	{0x0010, "TOOL_CHANGE_START"},
	{0x0020, "TOOL_CHANGE_END"},
	{0x0040, "WRITE_PROTECTION_DISABLE"},
	{0x0080, "WRITE_PROTECTION_ENABLE"},
}

// 40012 reads back the combined state of the drive (e.g. OFF+STOP for an
// idle drive). The manual is explicit that writes must be a single state but
// reads may be any combination.
func operationStateName(v uint16) string {
	if v == 0 {
		return "NONE"
	}
	parts := []string{}
	for _, b := range opStateBits {
		if v&b.bit != 0 {
			parts = append(parts, b.name)
		}
	}
	if len(parts) == 0 {
		return fmt.Sprintf("UNKNOWN(0x%04X)", v)
	}
	return strings.Join(parts, "+")
}

var alarmBits = []struct {
	bit  uint16
	name string
}{
	{0x0001, "tool_overheated"},
	{0x0002, "motor_drive_overheated"},
	{0x0004, "over_current"},
	{0x0008, "under_voltage"},
	{0x0010, "over_voltage"},
	{0x0020, "self_test_running"},
	{0x0040, "rpm_drop"},
	{0x0080, "high_current"},
	{0x0100, "tool_change_in_progress"},
	{0x0200, "tool_wiring_fault"},
	{0x0400, "factory_reset_mode"},
	{0x0800, "write_protection_disabled"},
}

func decodeAlarmFlags(v uint16) []interface{} {
	flags := []interface{}{}
	for _, a := range alarmBits {
		if v&a.bit != 0 {
			flags = append(flags, a.name)
		}
	}
	return flags
}

func (s *sander) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	command, ok := cmd["command"].(string)
	if !ok {
		return nil, errors.New("missing 'command' field")
	}

	switch command {
	case "start":
		rpm := s.defaultRPM
		if v, ok := cmd["rpm"]; ok {
			r, err := toRPM(v)
			if err != nil {
				return nil, err
			}
			rpm = r
		}
		if err := s.start(rpm); err != nil {
			return nil, err
		}
		return map[string]interface{}{"running": true, "rpm": int(rpm)}, nil

	case "stop":
		if err := s.stop(); err != nil {
			return nil, err
		}
		return map[string]interface{}{"running": false}, nil

	case "set_speed":
		v, ok := cmd["rpm"]
		if !ok {
			return nil, errors.New("set_speed requires 'rpm'")
		}
		rpm, err := toRPM(v)
		if err != nil {
			return nil, err
		}
		if err := s.setSpeed(rpm); err != nil {
			return nil, err
		}
		return map[string]interface{}{"rpm": int(rpm)}, nil

	case "status":
		return s.Status(ctx)

	default:
		return nil, fmt.Errorf("unknown command %q (supported: start, stop, set_speed, status)", command)
	}
}

func toRPM(v interface{}) (uint16, error) {
	switch n := v.(type) {
	case float64:
		return uint16(n), nil
	case int:
		return uint16(n), nil
	case int64:
		return uint16(n), nil
	default:
		return 0, fmt.Errorf("rpm must be a number, got %T", v)
	}
}

func (s *sander) Close(ctx context.Context) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Best-effort STOP+OFF so a crash or reconfigure never leaves the spindle spinning.
	if s.running {
		if err := s.stopLocked(); err != nil {
			s.logger.Warnf("stop on close failed: %v", err)
		}
	}
	return s.client.Close()
}
