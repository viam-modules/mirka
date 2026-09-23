package autochanger

// Process data bits, confirmed on the bench. Bit 4 is absent from the EMCS-ST
// manual -- Move Intermediate is a later firmware feature.
const (
	pdInAtZero         uint16 = 1 << 0 // State "In", at Lim(Ref)
	pdInAtRelease      uint16 = 1 << 1 // State "Out"
	pdInMoving         uint16 = 1 << 2 // State "Move"
	pdInReady          uint16 = 1 << 3 // State "Device"
	pdInAtIntermediate uint16 = 1 << 4
)

const pdOutQuitError uint16 = 1 << 2

// IsduPosImp holds the intermediate position target. Homing overwrites it with
// the full stroke, so it must be set before every intermediate move.
const IsduPosImp = 264 // 0x0108

// IsduPosActual holds the drive's actual position, in the same units. Unlike
// the process data bits it tracks continuously, so it is meaningful mid-travel.
const IsduPosActual = 288 // 0x0120

// Stroke, measured between the two mechanical end stops.
const (
	KnifeTravelCounts = 2601
	KnifeTravelMM     = 25.0
)

// PosImpPerMM is derived from the stroke
const PosImpPerMM = KnifeTravelCounts / KnifeTravelMM // 104.04

// PosImpOffsetMM is the gap still standing between knife and plate at the
// drive's zero.
const PosImpOffsetMM = 0.42

type Position int

const (
	PositionUnknown      Position = iota // no bit set: in motion, or never commanded
	PositionIn                           // at the retracted end stop, flush with the plate
	PositionOut                          // at the extended end stop
	PositionIntermediate                 // at the taught target, wherever that is
)

func (p Position) String() string {
	switch p {
	case PositionIn:
		return "in"
	case PositionOut:
		return "out"
	case PositionIntermediate:
		return "intermediate"
	default:
		return "unknown"
	}
}

type State struct {
	Position Position
	Moving   bool
	Ready    bool
	Raw      uint16
}

// DecodeState reads position and status out of a process data word. Position
// comes from hardware, never from what was last commanded, so it survives a
// restart.
func DecodeState(pd uint16) State {
	s := State{
		Moving: pd&pdInMoving != 0,
		Ready:  pd&pdInReady != 0,
		Raw:    pd,
	}
	switch {
	case pd&pdInAtZero != 0:
		s.Position = PositionIn
	case pd&pdInAtRelease != 0:
		s.Position = PositionOut
	case pd&pdInAtIntermediate != 0:
		s.Position = PositionIntermediate
	default:
		s.Position = PositionUnknown
	}
	return s
}

// OffsetForCounts converts drive units into knife displacement out from the
// retracted end, in mm. No offset term: both measure from the drive's zero.
func OffsetForCounts(counts int32) float64 {
	return float64(counts) / PosImpPerMM
}
