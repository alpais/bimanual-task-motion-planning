
for i = 1: size(Peel)
    data_m = [Peel{i}.passive.pos' Peel{i}.passive.ori'];
    sfname_master = sprintf('data_master_arm/data_pos_%0.3d.txt', i);
    save(sfname_master,'data_m','-ascii');
    clear data_m
    
    data_s = [Peel{i}.active.pos' Peel{i}.active.ori'];
    sfname_slave = sprintf('data_slave_arm/data_pos_%0.3d.txt', i);
    save(sfname_slave,'data_s','-ascii');
    clear data_s
end