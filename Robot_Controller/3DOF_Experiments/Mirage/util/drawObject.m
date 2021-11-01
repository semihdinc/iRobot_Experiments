function drawObject(varargin)

    if(nargin == 1)
        P = varargin{1};
        edgeColor = [0 0 0];
        type = 'pyr';
    elseif(nargin == 2)
        P = varargin{1};
        edgeColor = varargin{2};
        type = 'pyr';
    else
        P = varargin{1};
        edgeColor = varargin{2};
        type = varargin{3};
    end

    %%  
    if(strcmp(type, 'pyr') == 1)
        objectFaces = [1 2 3; 1 2 4; 1 3 4; 2 3 4];
    end
    
    patch('Vertices',P(1:3,:)','Faces',objectFaces,'FaceColor','none','EdgeColor',edgeColor);

end