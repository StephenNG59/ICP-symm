# ICP

## Requirements

* [PCL](https://github.com/PointCloudLibrary/pcl) (Point Cloud Library) 1.9.1 :

    [Github releases](https://github.com/PointCloudLibrary/pcl/releases) provides all-in-one-setup.exe.

## Usage

* --symm [source-file] [target-file] [optional: max_iters] [optional: diff]                   
    use symmetric object function to REGISTER source to target point clouds                 
* --p2p [source-file] [target-file] [optional: max_iters]

    use umeyama function to REGISTER source to target point clouds  

* -t [source-file] [output-file] [axis-x, y, z] [theta-in-degree] [translate-x, y, z]         

    TRANSFORM source file and save the result in output pcd file                            
  
* -td [source] [output-file] [axis-x, y, z] [theta-in-degree] [translate-x, y, z] [ratio]     
* -tdh [source] [output-file] [axis-x, y, z] [theta-in-degree] [translate-x, y, z] [ratio]    

    TRANSFORM, meanwhile DELETE *ratio* of the origin points.                               

    *-td* for randomly deletion, *-tdh* for hard deletion of the front part                 
  
* -cx/cy/cz [source] [output-file] xyz_cut                                                    

    CUT OUT the points whose x/y/z coord is greater than *xyz_cut*                          
    
* e.g.

    >ICP.exe --symm bunny-src.pcd bunny-tgt.pcd 50 0.001                                 
    
    >ICP.exe --p2p bunny-src.pcd bunny-tgt.pcd 10 --show-once                            
    
    >ICP.exe -t bunny.pcd bunny-src.pcd 1 0 1 30 10 -10 -5                         
    
    >ICP.exe -td bunny.pcd bunny-src.pcd 0 0 1 30 2 2 -5 0.4                       
    
    >ICP.exe -cy bunny.pcd bunny-cuty.pcd 2     # if y > 2, delete this point            