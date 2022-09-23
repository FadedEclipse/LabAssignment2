classdef KUKA < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-1 1 -1 1 0 1];   
   
    end
    
    methods%% Class for KUKA robot simulation
        function self = KUKA(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetKUKARobot();
            self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetKUKARobot
        % Given a name (optional), create and return a KUKA robot model
        function GetKUKARobot(self)
            pause(0.001);
            name = ['KUKA_',datestr(now,'yyyymmddTHHMMSSFFF')];
            L1 = Link('d',0.4,'a',0.025,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            L2 = Link('d',0,'a',0.455,'alpha',0,'offset',pi/2,'qlim',[deg2rad(-190),deg2rad(45)]);
            L3 = Link('d',0,'a',0.035,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-120),deg2rad(156)]);
            L4 = Link('d',0.460,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-185),deg2rad(185)]);
            L5 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-120),deg2rad(120)]);
            L6 = Link('d',0.08,'a',0,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(-350),deg2rad(350)]);
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end
