CREATE TABLE IF NOT EXISTS graph_nodes
                ( 
                  id bigserial primary key, 
                  pose_x double precision NOT NULL, 
                  pose_y double precision NOT NULL,  
                  pose_z double precision NOT NULL, 
                  pose_rotation double precision[4] NOT NULL, 
                  descriptors double precision[][] 
                )