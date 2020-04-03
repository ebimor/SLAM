more off;
clear all;
close all;

addpath('tools');

% load the graph into the variable g
% only leave one line uncommented

% simulation datasets
%load ../data/simulation-pose-pose.dat
%load ../data/simulation-pose-landmark.dat

% real-world datasets
%load ../data/intel.dat
load ../data/dlr.dat

fileID = fopen('data.g2o','w');
N = (length(g.x)/3)-1

%for i=0:N
%  fprintf(fileID,'VERTEX_SE2 %d %f %f %f \r\n',i,g.x(3*i+1:3*i+3));
%end

vertices = zeros(length(g.x));

for eid = 1:length(g.edges)
  edge = g.edges(eid);
  
  if (strcmp(edge.type, 'P') != 0)
  
      if (!vertices(edge.fromIdx))
        fprintf(fileID,'VERTEX_SE2 %d %f %f %f \r\n',edge.fromIdx,g.x(edge.fromIdx:edge.fromIdx+2));
        vertices(edge.fromIdx)=1;
      end
      
      if (!vertices(edge.toIdx))
        fprintf(fileID,'VERTEX_SE2 %d %f %f %f \r\n',edge.toIdx,g.x(edge.toIdx:edge.toIdx+2));
        vertices(edge.toIdx)=1;
      end
      
      I_triangular = [edge.information(1,1), edge.information(1,2), edge.information(1,3), edge.information(2,2), edge.information(2,3), edge.information(3,3)];
      fprintf(fileID, 'EDGE_SE2 %d %d %f %f %f %f %f %f %f %f %f \r\n', edge.fromIdx, edge.toIdx, edge.measurement ,I_triangular)
   
  elseif (strcmp(edge.type, 'L') != 0)
  
      if (!vertices(edge.fromIdx))
        fprintf(fileID,'VERTEX_SE2 %d %f %f %f \r\n',edge.fromIdx,g.x(edge.fromIdx:edge.fromIdx+2));
        vertices(edge.fromIdx)=1;
      end

      %the second pose landmark pose
      if (!vertices(edge.toIdx))
        fprintf(fileID,'VERTEX_XY %d %f %f \r\n',edge.toIdx,g.x(edge.toIdx:edge.toIdx+1));
        vertices(edge.toIdx)=1;
      end
      
      I_triangular = [edge.information(1,1), edge.information(1,2), edge.information(2,2)];
      fprintf(fileID, 'EDGE_SE2_XY %d %d %f %f %f %f %f \r\n', edge.fromIdx, edge.toIdx, edge.measurement, I_triangular)
      
  end
 end 
fclose(fileID);

