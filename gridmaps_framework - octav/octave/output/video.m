vidObj = VideoWriter('prmIII.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 15;
open(vidObj);

for t=1:686
    filename = sprintf('gridmap_%03d.png', t);
    if exist(filename, 'file') ~= 0
        img=imshow(filename)
        writeVideo(vidObj, getframe(gcf));
    end
end



close(vidObj);