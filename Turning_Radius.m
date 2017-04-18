%Turning Radius

values=world_outputs(185:700,4:5);

values(isnan(values(:,1)),:) = [];

circ_out=CircleFitByPratt(values)


     figure;
        plot(values(:,1),values(:,2),'*');
          hold on
        h=circle(circ_out(1),circ_out(2),circ_out(3));