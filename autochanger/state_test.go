package autochanger

import (
	"math"
	"testing"
)

func TestDecodeState(t *testing.T) {
	for _, tc := range []struct {
		name string
		pd   uint16
		want State
	}{
		{"at zero point", 0x0009, State{Position: PositionIn, Ready: true, Raw: 0x0009}},
		{"at release", 0x000A, State{Position: PositionOut, Ready: true, Raw: 0x000A}},
		{"at grip", 0x0018, State{Position: PositionIntermediate, Ready: true, Raw: 0x0018}},
		{"moving", 0x000C, State{Position: PositionUnknown, Moving: true, Ready: true, Raw: 0x000C}},
		{"ready, position unknown", 0x0008, State{Position: PositionUnknown, Ready: true, Raw: 0x0008}},
		{"not ready", 0x0000, State{Position: PositionUnknown, Raw: 0x0000}},
		{"leaving zero point", 0x000D, State{Position: PositionIn, Moving: true, Ready: true, Raw: 0x000D}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if got := DecodeState(tc.pd); got != tc.want {
				t.Fatalf("DecodeState(0x%04X) = %+v, want %+v", tc.pd, got, tc.want)
			}
		})
	}
}

func TestPositionString(t *testing.T) {
	if PositionIntermediate.String() != "intermediate" {
		t.Fatalf("got %q", PositionIntermediate.String())
	}
	if PositionUnknown.String() != "unknown" {
		t.Fatalf("got %q", PositionUnknown.String())
	}
}

func TestOffsetForCounts(t *testing.T) {
	// Readings taken off the drive at both end stops and two intermediate
	// targets, against the scale measured across the full stroke.
	for _, tc := range []struct {
		counts int32
		want   float64
	}{
		{0, 0},
		{71, 0.6824},
		{1499, 14.4079},
		{2601, KnifeTravelMM},
	} {
		if got := OffsetForCounts(tc.counts); math.Abs(got-tc.want) > 0.001 {
			t.Fatalf("OffsetForCounts(%d) = %v, want %v", tc.counts, got, tc.want)
		}
	}
}
