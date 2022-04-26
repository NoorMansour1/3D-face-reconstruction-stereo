function [TM,tri,a] = MESHfinal(Merge,res)
%% Import the points
Location=double(Merge.Location);
% Create the mesh 
ind=1:res:length(Location);
x=Location(ind,1);
y=Location(ind,2);
z=Location(ind,3);
tri = delaunay(x,y);
% calculating the edge
X=x(tri);
Y=y(tri);
a=zeros(size(X));
for cnt=1:3
a(:,cnt)=abs(diff(angle(bsxfun(@minus,X(:,[1:cnt-1 cnt+1:end]),X(:,cnt))+1i*bsxfun(@minus,Y(:,[1:cnt-1 cnt+1:end]),Y(:,cnt))),[],2));
end
%Visualizing
figure;
TM = trisurf(tri, x, y, z);
set(TM,'EdgeColor','none');
%set(TM,'FaceVertexCData',Merge.Color(ind,:));
%set(TM,'Facecolor','interp');
set(TM,'FaceColor','none');
set(TM,'EdgeColor','flat');
xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')
%axis([-250 250 -250 250 400 900])
set(gca,'xdir','reverse')
set(gca,'zdir','reverse')
daspect([1,1,1])
axis tight

grid off
%set(gca,'CameraPosition',[208 -50 7687])

end