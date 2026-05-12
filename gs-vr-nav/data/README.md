# Capture Data Requirements

The capture pipeline expects an outdoor image sequence collected from a mobile
phone, action camera, or comparable calibrated camera source.

## Input Format

- Images should be stored in one directory and use `.jpg`, `.jpeg`, or `.png`
  extensions.
- File names should preserve capture order where possible.
- Images should be suitable for COLMAP feature extraction: sharp, overlapping,
  and captured with enough viewpoint diversity for reconstruction.

## Required Metadata

Each image should include EXIF GPS metadata:

- Latitude in WGS84.
- Longitude in WGS84.
- Altitude when available.
- Capture timestamp when available.

Heading metadata is recommended but optional. If phone compass headings are
available, they can provide a useful initialization signal for geographic
alignment and debugging.

## Accuracy Expectations

The default minimum GPS accuracy target is 10 meters. Lower accuracy captures
may still reconstruct visually, but the initial alignment to OpenStreetMap data
will be less reliable and may require manual correspondences.
