clear all;
close all;
clc;
counter = 1;


QL_Matrix = csvread('QL_Matrix_22_7_16_1546.csv');
QL_Matrix(:,size(QL_Matrix,2)) = []; % get rid of the last column

row = size(QL_Matrix,1);
col = size(QL_Matrix,2);
[C,I] = max(QL_Matrix,[],1);

% find a goal
for i = 1:row
    for j = 1:col
        if QL_Matrix(i,j)>= max(C)
            i_G = i;
            j_G = j;
            break;
        end
    end
end

% finding a random component inside the matrix
row_rand = randi([1 row]);
col_rand = randi([1 col]);

state_init = QL_Matrix(row_rand,col_rand);
disp(['starting point => Q[',num2str(row_rand),'][',num2str(col_rand),'] = ',num2str(state_init)]);
state_curr = state_init;
state_next = 0;

vec_row(1) = row_rand;
vec_col(1) = col_rand;

row_indx = row_rand;
col_indx = col_rand;

% % row_indx = 7;
% % col_indx = 8;
% state_curr = QL_Matrix(row_indx,col_indx)



while row_indx ~= i_G && col_indx ~= j_G

if row_indx == 1 && col_indx == 1 % NW
    side_comp_NW = [QL_Matrix(row_indx + 1,col_indx) QL_Matrix(row_indx,col_indx + 1)];
    max_comp_NW = max(side_comp_NW);
    
    if QL_Matrix(row_indx + 1,col_indx) > state_curr && QL_Matrix(row_indx + 1,col_indx) == max_comp_NW
        % save the value...
        state_next = QL_Matrix(row_indx + 1,col_indx);
        % save the indeces...
        row_indx = row_indx + 1;
        col_indx = col_indx;
        
    else if QL_Matrix(row_indx,col_indx + 1) > state_curr && QL_Matrix(row_indx,col_indx + 1) == max_comp_NW
            state_next = QL_Matrix(row_indx,col_indx + 1);
            % save the indeces...
            row_indx = row_indx;
            col_indx = col_indx + 1;
        end
    end
else if row_indx == 1 && col_indx == col % NE
        side_comp_NE = [QL_Matrix(row_indx + 1,col_indx) QL_Matrix(row_indx,col_indx - 1)];
        max_comp_NE = max(side_comp_NE);
        
        if QL_Matrix(row_indx + 1,col_indx) > state_curr && QL_Matrix(row_indx + 1,col_indx) == max_comp_NE
            state_next = QL_Matrix(row_indx + 1,col_indx);
            % save the indeces...
            row_indx = row_indx + 1;
            col_indx = col_indx;
        
        else if QL_Matrix(row_indx,col_indx - 1) > state_curr && QL_Matrix(row_indx,col_indx - 1) == max_comp_NE
                state_next = QL_Matrix(row_indx,col_indx - 1);
                % save the indeces...
                row_indx = row_indx;
                col_indx = col_indx - 1;
        
            end
        end
    else if row_indx == row && col_indx == 1 % SW
            side_comp_SW = [QL_Matrix(row_indx - 1,col_indx) QL_Matrix(row_indx,col_indx + 1)];
            max_comp_SW = max(side_comp_SW);
            if QL_Matrix(row_indx - 1,col_indx) > state_curr && QL_Matrix(row_indx - 1,col_indx) == max_comp_SW
                state_next = QL_Matrix(row_indx - 1,col_indx);
                % save the indeces...
                row_indx = row_indx - 1;
                col_indx = col_indx;
        
            else if QL_Matrix(row_indx,col_indx + 1) > state_curr && QL_Matrix(row_indx,col_indx + 1) == max_comp_SW
                    state_next = QL_Matrix(row_indx,col_indx + 1);
                    % save the indeces...
                    row_indx = row_indx;
                    col_indx = col_indx + 1;
        
                end
            end
        else if row_indx == row && col_indx == col % SE
                side_comp_SE = [QL_Matrix(row_indx,col_indx - 1) QL_Matrix(row_indx - 1,col_indx)];
                max_comp_SE = max(side_comp_SE);
                
                if QL_Matrix(row_indx - 1,col_indx) > state_curr && QL_Matrix(row_indx - 1,col_indx) == max_comp_SE
                    state_next = QL_Matrix(row_indx - 1,col_indx);
                    % save the indeces...
                    row_indx = row_indx - 1;
                    col_indx = col_indx;
        
                else if QL_Matrix(row_indx,col_indx - 1) > state_curr && QL_Matrix(row_indx,col_indx - 1) == max_comp_SE
                        state_next = QL_Matrix(row_indx,col_indx - 1);
                        % save the indeces...
                        row_indx = row_indx;
                        col_indx = col_indx - 1;
        
                    end
                end
            else if row_indx == 1 % upp
                    side_comp_upp = [QL_Matrix(row_indx + 1,col_indx) QL_Matrix(row_indx,col_indx - 1) QL_Matrix(row_indx,col_indx + 1)];
                    max_comp_upp = max(side_comp_upp);
                    
                    if QL_Matrix(row_indx + 1,col_indx) > state_curr && QL_Matrix(row_indx + 1,col_indx) == max_comp_upp
                        state_next = QL_Matrix(row_indx + 1,col_indx);
                        % save the indeces...
                        row_indx = row_indx + 1;
                        col_indx = col_indx;
        
                    else if QL_Matrix(row_indx,col_indx - 1) > state_curr && QL_Matrix(row_indx,col_indx - 1) == max_comp_upp
                            state_next = QL_Matrix(row_indx,col_indx - 1);
                            % save the indeces...
                            row_indx = row_indx;
                            col_indx = col_indx - 1;
        
                        else if QL_Matrix(row_indx,col_indx + 1) > state_curr && QL_Matrix(row_indx,col_indx + 1) == max_comp_upp
                                state_next = QL_Matrix(row_indx,col_indx + 1);
                                % save the indeces...
                                row_indx = row_indx;
                                col_indx = col_indx + 1;
        
                            end
                        end
                    end
                else if row_indx == row % dwn
                        side_comp_dwn = [QL_Matrix(row_indx - 1,col_indx) QL_Matrix(row_indx,col_indx - 1) QL_Matrix(row_indx,col_indx + 1)];
                        max_comp_dwn = max(side_comp_dwn);
                        
                        if QL_Matrix(row_indx - 1,col_indx) > state_curr && QL_Matrix(row_indx - 1,col_indx) == max_comp_dwn
                            state_next = QL_Matrix(row_indx - 1,col_indx);
                            % save the indeces...
                            row_indx = row_indx - 1;
                            col_indx = col_indx;
        
                        else if QL_Matrix(row_indx,col_indx - 1) > state_curr && QL_Matrix(row_indx,col_indx - 1) == max_comp_dwn
                                
                                state_next = QL_Matrix(row_indx,col_indx - 1);
                                % save the indeces...
                                row_indx = row_indx;
                                col_indx = col_indx - 1;
        
                            else if QL_Matrix(row_indx,col_indx + 1) > state_curr && QL_Matrix(row_indx,col_indx + 1) == max_comp_dwn
                                    
                                    state_next = QL_Matrix(row_indx,col_indx + 1);
                                    % save the indeces...
                                    row_indx = row_indx;
                                    col_indx = col_indx + 1;
        
                                end
                            end
                        end
                    else if col_indx == 1 % lft
                            side_comp_lft = [QL_Matrix(row_indx - 1,col_indx) QL_Matrix(row_indx + 1,col_indx) QL_Matrix(row_indx,col_indx + 1)];
                            max_comp_lft = max(side_comp_lft);
                            
                            if QL_Matrix(row_indx - 1,col_indx) > state_curr && QL_Matrix(row_indx - 1,col_indx) == max_comp_lft
                                
                                state_next = QL_Matrix(row_indx - 1,col_indx);
                                % save the indeces...
                                row_indx = row_indx - 1;
                                col_indx = col_indx;
        
                            else if QL_Matrix(row_indx + 1,col_indx) > state_curr && QL_Matrix(row_indx + 1,col_indx) == max_comp_lft
                                    
                                    state_next = QL_Matrix(row_indx + 1,col_indx);
                                    % save the indeces...
                                    row_indx = row_indx + 1;
                                    col_indx = col_indx;
        
                                else if QL_Matrix(row_indx,col_indx + 1) > state_curr && QL_Matrix(row_indx,col_indx + 1) == max_comp_lft
                                        
                                        state_next = QL_Matrix(row_indx,col_indx + 1);
                                        % save the indeces...
                                        row_indx = row_indx;
                                        col_indx = col_indx + 1;
        
                                    end
                                end
                            end
                        else if col_indx == col % rgt
                                side_comp_rgt = [QL_Matrix(row_indx - 1,col_indx) QL_Matrix(row_indx + 1,col_indx) QL_Matrix(row_indx,col_indx - 1)];
                                max_comp_rgt = max(side_comp_rgt);
                                
                                if QL_Matrix(row_indx - 1,col_indx) > state_curr && QL_Matrix(row_indx - 1,col_indx) == max_comp_rgt
                                    
                                    state_next = QL_Matrix(row_indx - 1,col_indx);
                                    % save the indeces...
                                    row_indx = row_indx - 1;
                                    col_indx = col_indx;
        
                                else if QL_Matrix(row_indx + 1,col_indx) > state_curr && QL_Matrix(row_indx + 1,col_indx) == max_comp_rgt
                                        
                                        state_next = QL_Matrix(row_indx + 1,col_indx);
                                        % save the indeces...
                                        row_indx = row_indx + 1;
                                        col_indx = col_indx;
        
                                    else if QL_Matrix(row_indx,col_indx - 1) > state_curr && QL_Matrix(row_indx,col_indx - 1) == max_comp_rgt
                                            
                                            state_next = QL_Matrix(row_indx,col_indx - 1);
                                            % save the indeces...
                                            row_indx = row_indx;
                                            col_indx = col_indx - 1;
                                        end
                                    end
                                end
                            else % mid
                                side_comp_mid = [QL_Matrix(row_indx - 1,col_indx) QL_Matrix(row_indx + 1,col_indx) QL_Matrix(row_indx,col_indx - 1) QL_Matrix(row_indx,col_indx + 1)];
                                max_comp_mid = max(side_comp_mid);
                                
                                if QL_Matrix(row_indx - 1,col_indx) > state_curr && QL_Matrix(row_indx - 1,col_indx) == max_comp_mid
                                    
                                    state_next = QL_Matrix(row_indx - 1,col_indx);
                                    % save the indeces...
                                    row_indx = row_indx - 1;
                                    col_indx = col_indx;
        
                                else if QL_Matrix(row_indx + 1,col_indx) > state_curr && QL_Matrix(row_indx + 1,col_indx) == max_comp_mid
                                        
                                        state_next = QL_Matrix(row_indx + 1,col_indx);
                                        % save the indeces...
                                        row_indx = row_indx + 1;
                                        col_indx = col_indx;
        
                                    else if QL_Matrix(row_indx,col_indx - 1) > state_curr && QL_Matrix(row_indx,col_indx - 1) == max_comp_mid
                                            
                                            state_next = QL_Matrix(row_indx,col_indx - 1);
                                            % save the indeces...
                                            row_indx = row_indx;
                                            col_indx = col_indx - 1;
        
                                        else if QL_Matrix(row_indx,col_indx + 1) > state_curr && QL_Matrix(row_indx,col_indx + 1) == max_comp_mid
                                                
                                                state_next = QL_Matrix(row_indx,col_indx + 1);
                                                % save the indeces...
                                                row_indx = row_indx;
                                                col_indx = col_indx + 1;
        
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

state_curr = state_next;

counter = counter + 1;
vec_row(counter) = row_indx;
vec_col(counter) = col_indx;
end

vec_row(counter + 1) = i_G;
vec_col(counter + 1) = j_G;


plot(vec_col(1),vec_row(1),'ro','LineWidth',4);
hold on;
plot(vec_col,vec_row,'b--','LineWidth',2.5);
hold on;
plot(vec_col(size(vec_col,2)),vec_row(size(vec_row,2)),'ko','LineWidth',4)
axis([0 row + 1 0 col + 1]);
grid on
title('Grid');
ylabel('$ index_{\theta_{mar}} $','interpreter','latex','FontSize',17);
xlabel('$ index_{y_{mar}}$','interpreter','latex','FontSize',17);
l = legend('${Start}$','${\alpha = 0.1 , \gamma = 0.8}$','${Goal}$','Orientation','Horizontal');
set(l,'interpreter','latex','FontSize',14);



