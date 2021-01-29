function Movie = PlayMovie(Tree, Solution, RobotData, ObstacleData, Robot_parameter)

    Backtrace_path = Solution.BacktracePath;
    Backtrace_path = fliplr(Backtrace_path);
    Path_iter = length(Backtrace_path);
    
    State = Tree.State(:, Backtrace_path);
    
    pp = 1;
    
	figure
	set(gcf(), 'Renderer', 'opengl');
	axis([-2 12 -2 12]);
	axis square;
	hold on;
	PlotObstacle(ObstacleData)
    
    for k = 1:Path_iter
        if (mod(k, 8) == 0) || k == Path_iter || k == 1
            Robot_state = State(:, k);
           	
%             figure
%             set(gcf(), 'Renderer', 'opengl');
%             axis([-2 12 -2 12]);
%             axis square;
%             hold on;

            PlotRobot(Robot_state, Robot_parameter, RobotData);
%             PlotObstacle(ObstacleData)
            
            box on;
            set(gca,'xtick',[],'xticklabel',[])
            set(gca,'ytick',[],'yticklabel',[])
            
%             hold off;
            
            Movie(pp) = getframe;
            pp = pp + 1;
        end        
    end
    %movie(Movie,3);
%     hold off
end