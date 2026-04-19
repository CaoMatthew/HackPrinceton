"""
Main 3D perception pipeline.

Now includes:
- CLIP semantic labeling
- 3D segmentation
- Scene graph construction
- Affordance generation
- K2 action formatting
"""

from clip_model import CLIPModelWrapper
from segmentation_3d import SegmentAnything3D
from reconstruction import SceneReconstructor
from scene_representation import SceneGraph
from affordance_map import AffordanceMapGenerator


class PerceptionPipeline:
    def __init__(self):
        self.clip_model = CLIPModelWrapper()
        self.segmenter = SegmentAnything3D()
        self.reconstructor = SceneReconstructor()
        self.scene_graph = SceneGraph()
        self.affordance_generator = AffordanceMapGenerator()

    def process_input(self, rgb_frames, depth_frames):
        """
        Full pipeline:
        1. Reconstruct 3D scene
        2. Segment objects
        3. Label objects (CLIP)
        4. Build scene graph
        5. Generate affordances
        6. Export to K2
        """

        # -------------------------
        # Step 1 — 3D Reconstruction
        # -------------------------
        reconstructed_scene = self.reconstructor.build_scene(
            rgb_frames=rgb_frames,
            depth_frames=depth_frames
        )

        # -------------------------
        # Step 2 — 3D Segmentation
        # -------------------------
        segmented_objects = self.segmenter.segment_scene(
            scene_geometry=reconstructed_scene
        )

        # -------------------------
        # Step 3 — Semantic Labeling (CLIP)
        # -------------------------
        labeled_objects = self.clip_model.label_segments(
            segmented_objects=segmented_objects
        )

        # -------------------------
        # Step 4 — Scene Graph
        # -------------------------
        structured_scene = self.scene_graph.construct(
            labeled_objects=labeled_objects
        )

        # -------------------------
        # Step 5 — Affordances
        # -------------------------
        affordance_map = self.affordance_generator.generate_affordance_map(
            labeled_objects=labeled_objects
        )

        # -------------------------
        # Step 6 — K2 Actions
        # -------------------------
        k2_actions = self.affordance_generator.export_to_k2(
            affordance_map=affordance_map
        )

        return {
            "scene": structured_scene,
            "affordances": affordance_map,
            "k2_actions": k2_actions
        }


if __name__ == "__main__":
    pipeline = PerceptionPipeline()

    dummy_rgb_frames = "RGB_FRAME_SEQUENCE_PLACEHOLDER"
    dummy_depth_frames = "DEPTH_FRAME_SEQUENCE_PLACEHOLDER"

    result = pipeline.process_input(dummy_rgb_frames, dummy_depth_frames)

    print("Pipeline output:")
    print(result)