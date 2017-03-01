# zenith_map_maker
Ken's attempt at converting a point cloud to an image of the ground plan to be transmitted to the Zenith Visualizer

## Configs

### img_width
Size of x component of output image. Will map Point Cloud x (0, scan_depth) to image x(0, img_width)

### img_height
Size of y component of output image. Will map Point Cloud y (scan_width/2, -scan_width/2) to image y(0, img_height)
*note:* Flip of y to match perspective of ZED camera which has Y as left.

### scan_width
The width in meters that this node will search the Point cloud for corespoinding points in the image. Will look for points with y components between -scan_width/2 to scan_width/2

### scan_depth
The depth in meters that this node will search the Point cloud for corespoinding points in the image. Will look for points with x components between .01 and scan_depth

## Inputs

## Outputs
