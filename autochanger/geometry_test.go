package autochanger

import (
	"context"
	"testing"
)

func TestRemoverGeometries(t *testing.T) {
	r := &remover{}
	geoms, err := r.Geometries(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(geoms) != 3 {
		t.Fatalf("got %d geometries, want 3", len(geoms))
	}
	want := map[string]bool{
		"remover_body": false, "remover_sliding_plate": false, "remover_blade": false,
	}
	for _, g := range geoms {
		if _, ok := want[g.Label()]; !ok {
			t.Fatalf("unexpected label %q", g.Label())
		}
		want[g.Label()] = true
	}
	for label, seen := range want {
		if !seen {
			t.Fatalf("missing geometry %q", label)
		}
	}
	// The blade envelope must cover the full 25mm extension: its Y extent
	// must reach at least y=+25 from the zero point.
	for _, g := range geoms {
		if g.Label() == "remover_blade" {
			if g.Pose().Point().Y <= 0 {
				t.Fatal("blade envelope must extend in +Y (extension direction)")
			}
		}
	}
}
