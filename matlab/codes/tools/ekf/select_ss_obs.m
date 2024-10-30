function [ss_pool,selected_ss_obs]=select_ss_obs(mu_dc,sigma_dc,z,ss_pool,doa_w_diff_min)

selected_ss_obs = [];

selected_ss_obs_idx = 1;

for n=1:size(ss_pool,2)

    
    for m=1:size(z,2)
        
        if n==z(m).id
            
            if isempty(ss_pool(n).data)
                ss_pool(n).data = [];
                
                ss_pool(n).data.mu = mu_dc;
                ss_pool(n).data.sigma = sigma_dc;
                ss_pool(n).data.z = z;
                ss_pool(n).data.doa_w = wrapToPi(mu_dc(3)+z(m).bearing);
                
            else
                
                new_data.mu = mu_dc;
                new_data.sigma = sigma_dc;
                new_data.z = z;
                new_data.doa_w = wrapToPi(mu_dc(3)+z(m).bearing);
                
                for k=1:size(ss_pool(n).data,2)
                    
                    doa_w_diff = abs(wrapToPi(new_data.doa_w-ss_pool(n).data(k).doa_w));
                    if doa_w_diff>pi
                        doa_w_diff = 2*pi-doa_w_diff;
                    end
                    
                    if doa_w_diff>doa_w_diff_min
                        %selected_ss_obs_idx = selected_ss_obs_idx+1;
                        if selected_ss_obs_idx == 1
                            selected_ss_obs = ss_pool(n).data(k);
                            selected_ss_obs(2) = new_data;
                        else
                            selected_ss_obs(selected_ss_obs_idx) = ss_pool(n).data(k);
                            selected_ss_obs(selected_ss_obs_idx+1) = new_data;
                        end
                        selected_ss_obs_idx = selected_ss_obs_idx+2;
                        
                        ss_pool(n).data = [ss_pool(n).data(1:k-1) ss_pool(n).data(k+1:end)];
                        break;
                        
                    else
                        if k==size(ss_pool(n).data,2)
                            
                            ss_pool(n).data(k+1) = new_data;
                            
                        end
                    end
                    
                end
                
                new_data = [];
                
            end
            
        end
        
    end
    
    
end




end