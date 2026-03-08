function UFF = F_ZPETC(ZPETC,u_training_Array,y_training_Array,num_p,num_d)
    ZPETC_den = ZPETC.den{1};
    ZPETC_num = ZPETC.num{1};
    ZPETC_Layer = [ZPETC_num,-ZPETC_den(2:end-2*num_p-num_d)];
    UFF = u_training_Array;
    uff_shifted = u_training_Array(2:length(ZPETC_den)-2*num_p-num_d);
    y_shifted = y_training_Array(1)*ones(length(ZPETC_den), 1);
    for k = 4:(length(y_training_Array)-num_p-num_d)
        uff = ZPETC_Layer*[y_shifted;uff_shifted];
        uff_shifted = circshift(uff_shifted, 1);
        uff_shifted(1) = uff;    
        UFF(k) = uff;
        if k == length(y_training_Array)-num_p-num_d-1
            break
        end
        y_shifted = circshift(y_shifted, 1);
        y_shifted(1) = y_training_Array(k+num_p+num_d+1);
    end
end
