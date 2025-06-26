// Package obstaclesdistance uses an underlying camera to fulfill vision service methods, specifically
// GetObjectPointClouds, which performs several queries of NextPointCloud and returns a median point.
// The RDK version of this service is buggy and should be further investigated at some point. This implements
// the same functionality, and demonstrates the same buggy and laggy behavior as the RDK version.
package obstaclesdistance

import (
    "context"
    "image"
    "math"
    "sort"

    "github.com/golang/geo/r3"
    "github.com/pkg/errors"
    "go.opencensus.io/trace"

    "go.viam.com/rdk/components/camera"
    "go.viam.com/rdk/logging"
    "go.viam.com/rdk/pointcloud"
    "go.viam.com/rdk/resource"
    vision "go.viam.com/rdk/services/vision"
    "go.viam.com/rdk/spatialmath"
    "go.viam.com/rdk/utils"
    vis "go.viam.com/rdk/vision"
    "go.viam.com/rdk/vision/classification"
    "go.viam.com/rdk/vision/objectdetection"
    "go.viam.com/rdk/vision/viscapture"
	"go.viam.com/utils/rpc"
)

var Model = resource.NewModel("viam", "vision", "obstacles-distance")
var errUnimplemented = errors.New("obstacles distance service does not implement this method")

// DefaultNumQueries is the default number of times the camera should be queried before averaging.
const DefaultNumQueries = 10

// DistanceDetectorConfig specifies the parameters for the camera to be used
// for the obstacle distance detection service.
type DistanceDetectorConfig struct {
	resource.TriviallyValidateConfig
	NumQueries    int    `json:"num_queries"`
	DefaultCamera string `json:"camera_name"`
}

type obstacleDistanceService struct {
    resource.AlwaysRebuild
    name         resource.Name
    logger       logging.Logger
    segmenter    func(context.Context, camera.Camera) ([]*vis.Object, error)
    defaultCamera camera.Camera
    deps         resource.Dependencies
}

func init() {
	resource.RegisterService(vision.API, Model, resource.Registration[vision.Service, *DistanceDetectorConfig]{
		Constructor: func(
			ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger,
		) (vision.Service, error) {
			attrs, err := resource.NativeConfig[*DistanceDetectorConfig](conf)
			if err != nil {
				return nil, err
			}
			return registerObstacleDistanceDetector(ctx, conf.ResourceName(), attrs, deps)
		},
	})
}

// Validate ensures all parts of the config are valid.
func (config *DistanceDetectorConfig) Validate(path string) ([]string, []string, error) {
	var reqDeps []string
	var optDeps []string

	if config.NumQueries == 0 {
		config.NumQueries = DefaultNumQueries
	}
	if config.NumQueries < 1 || config.NumQueries > 20 {
		return nil, nil, errors.New("invalid number of queries, pick a number between 1 and 20")
	}
    if config.DefaultCamera != "" {
        reqDeps = append(reqDeps, config.DefaultCamera)
    }
	return reqDeps, optDeps, nil
}

func registerObstacleDistanceDetector(
    ctx context.Context,
    name resource.Name,
    conf *DistanceDetectorConfig,
    deps resource.Dependencies,
) (vision.Service, error) {
    _, span := trace.StartSpan(ctx, "service::vision::registerObstacleDistanceDetector")
    defer span.End()
    
    if conf == nil {
        return nil, errors.New("config for obstacles_distance cannot be nil")
    }

    segmenter := func(ctx context.Context, src camera.Camera) ([]*vis.Object, error) {
        // Your existing segmenter logic here
        clouds := make([]pointcloud.PointCloud, 0, conf.NumQueries)
        
        for i := 0; i < conf.NumQueries; i++ {
            nxtPC, err := src.NextPointCloud(ctx)
            if err != nil {
                return nil, err
            }
            if nxtPC.Size() == 0 {
                continue
            }
            clouds = append(clouds, nxtPC)
        }
        
        if len(clouds) == 0 {
            return nil, errors.New("none of the input point clouds contained any points")
        }

        median, err := medianFromPointClouds(ctx, clouds)
        if err != nil {
            return nil, err
        }

        vector := pointcloud.NewVector(median.X, median.Y, median.Z)
        pt := spatialmath.NewPoint(vector, "obstacle")

        pcToReturn := pointcloud.NewBasicEmpty()
        basicData := pointcloud.NewBasicData()
        err = pcToReturn.Set(vector, basicData)
        if err != nil {
            return nil, err
        }

        toReturn := make([]*vis.Object, 1)
        toReturn[0] = &vis.Object{PointCloud: pcToReturn, Geometry: pt}

        return toReturn, nil
    }
	var defaultCam camera.Camera
	var err error
    if conf.DefaultCamera != "" {
        defaultCam, err = camera.FromDependencies(deps, conf.DefaultCamera)
        if err != nil {
            return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
        }
    }

    myObsDist := &obstacleDistanceService{
        name:          name,
        logger:        logging.NewLogger("obstacles-distance"),
        segmenter:     segmenter,
        defaultCamera: defaultCam,
        deps:          deps,
    }

    return myObsDist, nil
}

func medianFromPointClouds(ctx context.Context, clouds []pointcloud.PointCloud) (r3.Vector, error) {
	var results [][]r3.Vector // a slice for each process, which will contain a slice of vectors
	err := utils.GroupWorkParallel(
		ctx,
		len(clouds),
		func(numGroups int) {
			results = make([][]r3.Vector, numGroups)
		},
		func(groupNum, groupSize, from, to int) (utils.MemberWorkFunc, utils.GroupWorkDoneFunc) {
			closestPoints := make([]r3.Vector, 0, groupSize)
			return func(memberNum, workNum int) {
					closestPoint := getClosestPoint(clouds[workNum])
					closestPoints = append(closestPoints, closestPoint)
				}, func() {
					results[groupNum] = closestPoints
				}
		},
	)
	if err != nil {
		return r3.Vector{}, err
	}
	candidates := make([]r3.Vector, 0, len(clouds))
	for _, r := range results {
		candidates = append(candidates, r...)
	}
	if len(candidates) == 0 {
		return r3.Vector{}, errors.New("point cloud list is empty, could not find median point")
	}
	return getMedianPoint(candidates), nil
}

func getClosestPoint(cloud pointcloud.PointCloud) r3.Vector {
	minDistance := math.MaxFloat64
	minPoint := r3.Vector{}
	cloud.Iterate(0, 0, func(pt r3.Vector, d pointcloud.Data) bool {
		dist := pt.Norm2()
		if dist < minDistance {
			minDistance = dist
			minPoint = pt
		}
		return true
	})
	return minPoint
}

// to calculate the median, will need to sort the vectors by distance from origin.
func sortVectors(v []r3.Vector) {
	sort.Sort(points(v))
}

type points []r3.Vector

func (p points) Len() int           { return len(p) }
func (p points) Swap(i, j int)      { p[i], p[j] = p[j], p[i] }
func (p points) Less(i, j int) bool { return p[i].Norm2() < p[j].Norm2() }

func getMedianPoint(pts []r3.Vector) r3.Vector {
	sortVectors(pts)
	index := (len(pts) - 1) / 2
	return pts[index]
}

func (s *obstacleDistanceService) GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vis.Object, error) {
    var cam camera.Camera
    var err error

    if cameraName != "" {
        cam, err = camera.FromDependencies(s.deps, cameraName)
        if err != nil {
            return nil, err
        }
    } else if s.defaultCamera != nil {
        cam = s.defaultCamera
    } else {
        return nil, errors.New("no camera specified")
    }

    return s.segmenter(ctx, cam)
}

func (s *obstacleDistanceService) CaptureAllFromCamera(ctx context.Context, cameraName string, captureOptions viscapture.CaptureOptions, extra map[string]interface{}) (viscapture.VisCapture, error) {
    var cam camera.Camera
    var err error

    if cameraName != "" {
		cam, err = camera.FromDependencies(s.deps, cameraName)
		if err != nil {
			return viscapture.VisCapture{}, err
		}
	} else if s.defaultCamera != nil {
		cam = s.defaultCamera
	} else {
		return viscapture.VisCapture{}, errors.New("no camera specified")
	}

	result := viscapture.VisCapture{}

	if captureOptions.ReturnImage {
		img, err := camera.DecodeImageFromCamera(ctx, "", nil, cam)
		if err != nil {
			return viscapture.VisCapture{}, err
		}
		result.Image = img
	}

	if captureOptions.ReturnObject {
		objects, err := s.GetObjectPointClouds(ctx, cameraName, extra)
		if err != nil {
			return viscapture.VisCapture{}, err
		}
		result.Objects = objects
	}

	result.Detections = []objectdetection.Detection{}
	result.Classifications = classification.Classifications{}

	return result, nil
}

func (s *obstacleDistanceService) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (vision.Service, error) {
	return nil, errUnimplemented
}

func (s *obstacleDistanceService) Detections(ctx context.Context, img image.Image, extra map[string]interface{}) ([]objectdetection.Detection, error) {
    return nil, errUnimplemented
}

func (s *obstacleDistanceService) DetectionsFromCamera(ctx context.Context, cameraName string, extra map[string]interface{}) ([]objectdetection.Detection, error) {
    return nil, errUnimplemented
}

func (s *obstacleDistanceService) Classifications(ctx context.Context, img image.Image, count int, extra map[string]interface{}) (classification.Classifications, error) {
    return nil, errUnimplemented
}

func (s *obstacleDistanceService) ClassificationsFromCamera(ctx context.Context, cameraName string, count int, extra map[string]interface{}) (classification.Classifications, error) {
    return nil, errUnimplemented
}

func (s *obstacleDistanceService) GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error) {
    return &vision.Properties{
        ClassificationSupported: false,
        DetectionSupported:      false,
        ObjectPCDsSupported:     true,
    }, nil
}

func (s *obstacleDistanceService) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
    return nil, errUnimplemented
}

func (s *obstacleDistanceService) Name() resource.Name {
    return s.name
}

func (s *obstacleDistanceService) Close(ctx context.Context) error {
    return nil
}
