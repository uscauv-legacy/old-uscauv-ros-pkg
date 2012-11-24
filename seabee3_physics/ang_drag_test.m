function ang_drag_test( rect_dims, samples )

n = samples
dim_x = rect_dims(1)
dim_y = rect_dims(2)
sample_pts_u = zeros( n, 2 )
sample_pts_v = zeros( n, 2 )
sample_vecs_u = zeros( n, 2 )
sample_vecs_v = zeros( n, 2 )
drag_unit_vecs = zeros( 2 * n, 2 )

# boundary points for the rectangle
dim_max = max( dim_x, dim_y ) / 2 + 1
bounds_graph = [ dim_max dim_max; -dim_max dim_max; -dim_max -dim_max; dim_max -dim_max; dim_max dim_max ]
bounds_points = [ dim_x/2 dim_y/2; -dim_x/2 dim_y/2; -dim_x/2 -dim_y/2; dim_x/2 -dim_y/2; dim_x/2 dim_y/2 ]

for i = 1:n
	# generate points along edges
	# left vertical edge
	sample_pts_u(i,:) = [ dim_x/2, -(2*i-1)*dim_y/( 4 * n ) ]
	# top horizontal edge
	sample_pts_v(i,:) = [ (2*i-1)*dim_x/( 4 * n ), dim_y / 2 ]
end

# combine u/v points
sample_pts = [ sample_pts_u; sample_pts_v ]

for i = 1:rows( sample_pts_u )
	# generate sample vectors for vertical and horizontal faces
	sample_vec_u = sample_pts_u(i,:) * [0 1; -1 0 ]
	sample_vec_v = sample_pts_v(i,:) * [0 1; -1 0 ]

	# save the normalized sample vectors
	sample_vecs_u(i,:) = sample_vec_u / norm( sample_vec_u )
	sample_vecs_v(i,:) = sample_vec_v / norm( sample_vec_v )
	
	# the unit drag vector along the x axis 
	drag_unit_vecs(i,:) = [ -sample_vecs_u(i,1), 0 ]
	# the unit drag vector along the y axis
	drag_unit_vecs(i+rows(sample_pts_u),:) = [ 0, -sample_vecs_v(i,2) ]
end

# combine u/v vecs
sample_vecs = [ sample_vecs_u; sample_vecs_v ]

# mirror values to other corners (origin symmetry)
sample_vecs = [ sample_vecs; -sample_vecs ]
sample_pts = [ sample_pts; -sample_pts ]
drag_unit_vecs = [drag_unit_vecs; -drag_unit_vecs ]

# transform from 3D x/y plane to 2D x/y plane
sample_vecs *= [ 0 1; -1 0 ]
sample_pts *= [ 0 1; -1 0 ]
drag_unit_vecs *= [ 0 1; -1 0 ]
bounds_graph *= [ 0 1; -1 0 ]
bounds_points *= [ 0 1; -1 0 ]

# draw
clf
hold on
plot( bounds_graph(:,1), bounds_graph(:,2), "-@3" )
plot( bounds_points(:,1), bounds_points(:,2), "-@3" )
plot( sample_pts(:,1), sample_pts(:,2), "@1" )

for i = 1:rows( sample_vecs )
	quiver( sample_pts(i,1), sample_pts(i,2), sample_vecs(i,1), sample_vecs(i,2), "2" )
	quiver( sample_pts(i,1), sample_pts(i,2), drag_unit_vecs(i,1), drag_unit_vecs(i,2), "1" )
end