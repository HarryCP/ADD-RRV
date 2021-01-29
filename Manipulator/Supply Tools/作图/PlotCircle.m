function PlotCircle(Origin, Type, LineWidth)
    t = 0:0.001:2*pi;
    r = 0.1;
    cir_x = r*cos(t) + Origin(1);
    cir_y = r*sin(t) + Origin(2);
    plot(cir_x, cir_y, Type, 'LineWidth', LineWidth);
    fill(cir_x, cir_y, Type);
end
