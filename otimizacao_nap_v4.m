%calcula quantidade de zonas no mapa (em cada direção)

%o total de zonas é qtde_zonas^2, assim como "area" é só a dimensão em uma
%direção e a área total é area^2 (por mais contra-intuitivo que seja)

%para limitar esforço computacional (número arbitrado para limitar combinações) 
qtde_zonas = 11 - nap;
tamanho_zona = area/qtde_zonas;

%determina as zonas em uma lista corrente
zona_indice=1;
zona_lista = zeros(qtde_zonas^2,2);
for i=1:qtde_zonas
    for j=1:qtde_zonas
        
        zona_lista(zona_indice,1)=i;
        zona_lista(zona_indice,2)=j;
        zona_indice = zona_indice+1;
        
    end
end

%atribui uma zona a cada nodo
zona_do_nodo = zeros(ndisp,2);
for i=(nap+1):n
    
    zona_do_nodo(i,:) = [ceil(pos(i,1)/tamanho_zona) ceil(pos(i,2)/tamanho_zona)];
    
end

etx_entre_zonas = Inf(qtde_zonas^2,qtde_zonas^2);

%varrendo todos os nodos
for nodos=(nap+1):n
    
    zona_testada = [zona_do_nodo(nodos,1) zona_do_nodo(nodos,2)];
    
    %varrendo todos os vizinhos dos nodos
    for nodos_vizinhos=(nap+1):n
        
        zona_vizinha = [zona_do_nodo(nodos_vizinhos,1) zona_do_nodo(nodos_vizinhos,2)];
        
        %se os nodos nao estao na mesma zona
        if (    zona_testada(1) - zona_vizinha(1)   )^2 + (    zona_testada(2) - zona_vizinha(2)   )^2 > 0
            
            %transforma as zonas em coordenadas (x,y) em uma zona indicada
            %por índice único, conforme a lista corrente criada no começo
            a=(zona_testada(1)-1)*qtde_zonas+zona_testada(2);
            b=(zona_vizinha(1)-1)*qtde_zonas+zona_vizinha(2);
            
            %se o etx entre os nodos é menor que o rank atual entre essas
            %zonas
            if etx(nodos,nodos_vizinhos) < etx_entre_zonas(a,b)

                %sobrescreve o rank da zona
                etx_entre_zonas(a,b) = etx(nodos,nodos_vizinhos);
                etx_entre_zonas(b,a) = etx(nodos,nodos_vizinhos);
               
            end
            
        end
        
    end
    
end

%calcula a matriz de combinações possíveis entre zonas para a qtde de naps
%presentes
qtde_total_zonas = qtde_zonas^2;
possib = nchoosek(1:qtde_total_zonas,nap);

%calcula o total de combinações possíveis;
possib_qty = nchoosek(qtde_total_zonas,nap);

%salva aqui o rank total das zonas a cada posiçao potencial de nap
rank_total = zeros(possib_qty,1);

for iteracoes=1:possib_qty

    %reseta rank das zonas para nova iteraçao
    rank_zonas = zeros(qtde_zonas^2,1)+999;

    %zera o rank da zona a qual está se testando possuir um NAP
    for zerar_naps=1:nap
        rank_zonas(possib(iteracoes,zerar_naps))=0;
    end

    %repetindo por um número arbitrariamente alto de iterações
    for aux=1:200

            %varre todas as zonas
            for varre_zonas=1:qtde_zonas^2

                %varre todas as zonas vizinhas até achar alguma cuja rank+etx seja
                %melhor que o seu atual
                for varre_zonas_vizinhas=1:qtde_zonas^2

                    %se nao está comparando ela consigo mesma
                    if varre_zonas ~= varre_zonas_vizinhas

                        %se o rank+etx é melhor que seu rank atual
                        if rank_zonas(varre_zonas) > rank_zonas(varre_zonas_vizinhas)+etx_entre_zonas(varre_zonas,varre_zonas_vizinhas)

                            %atualiza o rank da zona avaliada como o rank da zona encontrada mais o
                            %etx entre elas, que nem no RPL
                            rank_zonas(varre_zonas) = rank_zonas(varre_zonas_vizinhas)+etx_entre_zonas(varre_zonas,varre_zonas_vizinhas);

                        end

                    end

                end


            end

    end

    rank_total(iteracoes)=sum(rank_zonas);

end

%resultado expoe qual a melhor zona indicada para se posicionar NAP
[menor_rank ,   possib_otima] = min(rank_total);
disp(['Melhor rank total = ',num2str(menor_rank)]);
disp(['Melhor combinação = ',num2str(possib_otima)]);

for i=1:nap
    
    disp(['Melhor zona para NAP ',num2str(i),' = ',num2str( possib( possib_otima,i  )  )]);
    coordenadas_indicadas = [zona_lista(possib( possib_otima,i),1) , zona_lista(possib( possib_otima,i),2)];
    disp(['Posição recomendada para NAP',num2str(i),'= [',num2str(coordenadas_indicadas(1)*tamanho_zona-tamanho_zona/2),' , ',num2str(coordenadas_indicadas(2)*tamanho_zona-tamanho_zona/2),']']);
    
end


    

