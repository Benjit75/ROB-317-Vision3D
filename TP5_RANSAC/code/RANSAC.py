#
#
#      0===========================0
#      |    MAREVA 3D Modelling    |
#      0===========================0
#
#
# ----------------------------------------------------------------------------------------------------------------------
#
#      Script of the practical session. Plane detection by RANSAC
#
# ----------------------------------------------------------------------------------------------------------------------
#
#      Hugues THOMAS - 19/09/2018
#


# ----------------------------------------------------------------------------------------------------------------------
#
#          Imports and global variables
#      \**********************************/
#


# Import numpy package and name it "np"
import numpy as np

# Import functions to read and write ply files
from utils.ply import write_ply, read_ply

# Import time package
import time


# ----------------------------------------------------------------------------------------------------------------------
#
#           Functions
#       \***************/
#
#
#   Here you can define useful functions to be used in the main
#


def compute_plane(points: np.ndarray) -> (np.ndarray, np.ndarray):
    """
    Compute the plane passing through 3 points
    Parameters
    ----------
    points: (3, 3) array of float

    Returns
    -------
    point: (3,) array of float
        A point on the plane chosen as reference arbitrarily
    normal: (3,) array of float
        The normal vector of the plane
    """
    # Choose the first point as the reference point
    ref_point = points[0]

    # Compute two vectors from the three points
    vec1 = points[1] - ref_point
    vec2 = points[2] - ref_point

    # Compute the normal vector using the cross product
    normal = np.cross(vec1, vec2)

    # Normalize the normal vector
    normal /= np.linalg.norm(normal)
    
    return ref_point, normal


def in_plane(points: np.ndarray, ref_pt: np.ndarray, normal: np.ndarray, threshold_in: float=0.1) -> np.ndarray:
    """
    Return a boolean mask of points within the threshold distance to the plane.

    Parameters
    ----------
    points: (N, 3) array of float
        Array of points to check.
    ref_pt: (3,) array of float
        A point on the plane.
    normal: (3,) array of float
        The normal vector of the plane.
    threshold_in: float
        Distance threshold to consider a point as in the plane. Default is 0.1.

    Returns
    -------
    indices: (N,) array of bool
        Boolean mask of points within the threshold distance to the plane.
    """
    # Compute the distance of each point to the plane
    distances = np.abs((points - ref_pt)@normal)

    # Return a boolean mask of points within the threshold distance
    indices = distances < threshold_in

    return indices


def RANSAC(points: np.ndarray, NB_RANDOM_DRAWS: int=100, threshold_in: int=0.1) -> (np.ndarray, np.ndarray):
    """
    Return the prominent plane in a point cloud using RANSAC.

    Parameters
    ----------
    points: (N, 3) array of float
        Array of points in the point cloud.
    NB_RANDOM_DRAWS: int
        Number of random draws to perform. Default is 100.
    threshold_in: float
        Distance threshold to consider a point as in the plane. Default is 0.1.

    Returns
    -------
    best_ref_pt: (3,) array of float
        A point on the best plane.
    best_normal: (3,) array of float
        The normal vector of the best plane.
    """
    best_ref_pt = np.zeros((3,1))
    best_normal = np.zeros((3,1))
    max_votes = 0

    for _ in range(NB_RANDOM_DRAWS):
        # Randomly sample 3 points
        sample_points = points[np.random.choice(points.shape[0], 3, replace=False)]

        # Compute the plane defined by these 3 points
        ref_pt, normal = compute_plane(sample_points)

        # Count votes (points within the threshold distance to the plane)
        votes = np.sum(in_plane(points, ref_pt, normal, threshold_in))

        # Update the best plane if the current one has more votes
        if votes > max_votes:
            max_votes = votes
            best_ref_pt = ref_pt
            best_normal = normal
                
    return best_ref_pt, best_normal


def multi_RANSAC(points: np.ndarray, NB_RANDOM_DRAWS: int=100, threshold_in: float=0.1, NB_PLANES: int=2) -> (np.ndarray, np.ndarray, np.ndarray):
    """
    Return the best planes in a point cloud using RANSAC.
    Parameters
    ----------
    points: (N, 3) array of float
        Array of points in the point cloud.
    NB_RANDOM_DRAWS: int
        Number of random draws to perform. Default is 100.
    threshold_in: float
        Distance threshold to consider a point as in the plane. Default is 0.1.
    NB_PLANES: int
        Number of planes to find. Default is 2.

    Returns
    -------
    plane_inds: (N,) array of int
        Indices of the points in the best planes.
    remaining_inds: (N,) array of int
        Indices of the points not in the best planes.
    plane_labels: (N,) array of int
        Labels of the planes for each point.
    """
    plane_inds_list = []
    remaining_inds = np.arange(len(points))
    plane_labels = np.full(len(points), -1, dtype=int)

    for i in range(NB_PLANES):
        # Apply RANSAC to find the best plane
        best_ref_pt, best_normal = RANSAC(points[remaining_inds], NB_RANDOM_DRAWS, threshold_in)

        # Find points in the plane
        points_in_plane = in_plane(points[remaining_inds], best_ref_pt, best_normal, threshold_in)
        plane_inds = remaining_inds[points_in_plane.nonzero()[0]]

        # Store the indices of the points in the plane
        plane_inds_list.append(plane_inds)
        plane_labels[plane_inds] = i

        # Update remaining points
        remaining_inds = remaining_inds[(1 - points_in_plane).nonzero()[0]]

    plane_inds = np.concatenate(plane_inds_list)
    
    return plane_inds, remaining_inds, plane_labels


# ----------------------------------------------------------------------------------------------------------------------
#
#           Main
#       \**********/
#
# 
#   Here you can define the instructions that are called when you execute this file
#

if __name__ == '__main__':

    # Load point cloud
    # ****************
    #
    #   Load the file '../data/indoor_scan.ply'
    #   (See read_ply function)
    #

    # Path of the file
    file_path = '../data/indoor_scan.ply'

    # Load point cloud
    data = read_ply(file_path)

    # Concatenate data
    points = np.vstack((data['x'], data['y'], data['z'])).T
    colors = np.vstack((data['red'], data['green'], data['blue'])).T
    N = len(points)

    # Computes the plane passing through 3 randomly chosen points
    # ***********************************************************
    #

    if True:

        # Define parameter
        threshold_in = 0.1

        # Take randomly three points
        pts = points[np.random.randint(0, N, size=3)]

        # Computes the plane passing through the 3 points
        t0 = time.time()
        ref_pt, normal = compute_plane(pts)
        t1 = time.time()
        print('plane computation done in {:.3f} seconds'.format(t1 - t0))

        # Find points in the plane and others
        t0 = time.time()
        points_in_plane = in_plane(points, ref_pt, normal, threshold_in)
        t1 = time.time()
        print('plane extraction done in {:.3f} seconds'.format(t1 - t0))
        plane_inds = points_in_plane.nonzero()[0]

        # Save the 3 points and their corresponding plane for verification
        pts_clr = np.zeros_like(pts)
        pts_clr[:, 0] = 1.0
        write_ply('../triplet.ply',
                  [pts, pts_clr],
                  ['x', 'y', 'z', 'red', 'green', 'blue'])
        write_ply('../triplet_plane.ply',
                  [points[plane_inds], colors[plane_inds]],
                  ['x', 'y', 'z', 'red', 'green', 'blue'])

    # Computes the best plane fitting the point cloud
    # ***********************************
    #

    if True:

        # Define parameters of RANSAC
        NB_RANDOM_DRAWS = 130
        threshold_in = 0.05

        # Find best plane by RANSAC
        t0 = time.time()
        best_ref_pt, best_normal = RANSAC(points, NB_RANDOM_DRAWS, threshold_in)
        t1 = time.time()
        print('RANSAC done in {:.3f} seconds'.format(t1 - t0))

        # Find points in the plane and others
        points_in_plane = in_plane(points, best_ref_pt, best_normal, threshold_in)
        plane_inds = points_in_plane.nonzero()[0]
        remaining_inds = (1-points_in_plane).nonzero()[0]

        # Count the number of points in the prominent plane
        print(f'The prominent plane represents {len(plane_inds)} points over the {N} points.')

        # Save the best extracted plane and remaining points
        write_ply('../best_plane.ply',
                  [points[plane_inds], colors[plane_inds]],
                  ['x', 'y', 'z', 'red', 'green', 'blue'])
        write_ply('../remaining_points.ply',
                  [points[remaining_inds], colors[remaining_inds]],
                  ['x', 'y', 'z', 'red', 'green', 'blue'])

    # Find multiple planes in the cloud
    # *********************************
    #

    if True:

        # Define parameters of multi_RANSAC
        NB_RANDOM_DRAWS = 200
        threshold_in = 0.05
        NB_PLANES = 5

        # Recursively find best plane by RANSAC
        t0 = time.time()
        plane_inds, remaining_inds, plane_labels = multi_RANSAC(points, NB_RANDOM_DRAWS, threshold_in, NB_PLANES)
        t1 = time.time()
        print('\nmulti RANSAC done in {:.3f} seconds'.format(t1 - t0))

        # Save the best planes and remaining points
        write_ply('../best_planes.ply',
                  [points[plane_inds], colors[plane_inds], plane_labels[plane_inds].astype(np.int32)],
                  ['x', 'y', 'z', 'red', 'green', 'blue', 'plane_label'])
        write_ply('../remaining_points_.ply',
                  [points[remaining_inds], colors[remaining_inds]],
                  ['x', 'y', 'z', 'red', 'green', 'blue'])

        print('Done')
