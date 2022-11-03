%%
% 2022
% UFRGS - PPGEE
% Mestrado em Sistemas de Automa��o
% Autor: Dylan M. Timm
% T�cnica de Distruibui��o de M�ltiplos Pontos de Acesso para Otimizar
% Convergecast de Redes Industriais Sem Fio de Alta Escalabilidade

%% Inicializa��o

%est� se gerando uma rede nova ou se est� importando uma j� gerada?
% import = false;

%limpar variaveis e comandos
if import == false
    clear all;
    clc;
    import = false;
end

%% Declara��o de Par�metros de Simula��o

%numero de dispositivos totais
n = 300;   

%numero de pontos de acesso
nap = 3;        

%numero de dispositivos de campo
ndisp = n-nap;            

%lateral da �rea de disposi��o dos dispositivo
area = ceil(sqrt(n))*3;                        

%caso se esteja gerando rede nova
if import == false
    
    %posi��o aleatoria de cada dispositivo
    pos=randi(area,n,2);
    
    %pre-inicializacao de arrays
    prx_rand=zeros(n,n);
end

%calcula qtde de pares entre dispositivos
pares = n*(n-1)/2;  

%velocidade da luz em m/s
c = 299792458;  

%frequ�ncia de opera��o do WirelessHART em Hz
f=2.4*10.^9; %em Hz

%wavelength em metros
lambda = c/f;

%pot�ncia de transmiss�o do dispositivo WirelessHART em dB
ptx = 10;

%range de intefer�ncias externas aleat�rias
interf = 20; %fator aleatorio
interf_const= 1.5; %fator de deprecia��o em todos os links em fun�ao da distancia

%limite superior de pot�ncia recebida em dB para transmiss�o com 99% de
%chance de sucesso
limite_sup = -35;

%limite inferior de pot�ncia recebida em dB para transmiss�o com 99% de
%chance de sucesso
limite_inf = -100;

%limite superior do rank inicial de RPL (arbitrariamente grande)
rank_sup = 999999;

%quantidade de itera��es do algoritmo de RPL (precisa ser par)
rpl_iter = 50;

%limite superior de ETX para que dois vizinhos possam ser considerados
%"linkaveis". � um array porque se um dispositivo possuir poucos vizinhos
%poss�veis, ele pode dar folga ao seu limite 
etx_thresh = zeros(n,1);
etx_thresh = etx_thresh + 2;

%peso do AP Load Balancing no algoritmo de RPL. Recalcular em fun��o dos
%maiores ranks identificados na rede
load_b = 12;

%numero de ciclos de superframes para simula��o da rede
n_ciclos = 100;

%numero de superframes diferentes para a rede
n_superframes = nap;

%Time to Live (quanto tempo um pacote ainda tem utilidade e ainda vale a
%pena buscar rote�-lo ao destino) medido em numero de ciclos
ttl = 10;

%tamanho do buffer de mensagens de cada dispositivo
tam_buffer = 10;

%redundancia de links (quantos slots iguais entre vizinhos estao presentes
%em cada superframe
red_link = 4;

%redundancia de rota (quantas rotas alternativas cada dispositivo possui ate o AP) 
red_rota = 0;

%fator para for�ar maiores interfer�ncias externas em momentos arbitr�rios
%do funcionamento da rede
fator_dif_transm = 10;

%% Packet Delivery Ratio (PDR) e Expected Transmission Count (ETX)

%pre-aloca��es
pdr=zeros(n);
etx=zeros(n);
prx=zeros(n);
prx_friis=zeros(n,n);
dist=zeros(n,n);

%packet delivery ratio (forma��o com dist�ncia & interfer�ncias aleat�rias)
for i=1:n                             
    for j=1:n
        if i<j
            %dist�ncia entre dois pontos cartesianos
            dist(i,j) = sqrt((pos(i,1)-pos(j,1)).^2+(pos(i,2)-pos(j,2)).^2); 
            
            %impacto da dist�ncia sobre pdr, usando modelo de Friis (prx =
            %pot�ncia percebida no receptor)
            prx_friis(i,j) = ptx + 20*log10(lambda/(4*pi*dist(i,j)));
            
            %impacto de interfer�ncias aleat�rias sobre o pdr
            %para se preservar as interferncias aleatorias da rede
            %pre-gerada
            if import == false
                prx_rand(i,j) = randi(interf,1,1);
            end
            
            %pot�ncia percebida resultante
            prx(i,j) = prx_friis(i,j) - prx_rand(i,j) - interf_const*dist(i,j);
            
            %pdr 
            if prx(i,j) > limite_sup
                pdr(i,j) = 0.99;
            elseif prx(i,j) < limite_inf
                pdr(i,j) = 0;
            else
                %progress�o linear entre -100 dBm equivalendo a 0% e -10
                %dBm equivalendo 100%
                pdr(i,j) = (prx(i,j) - limite_inf)/(limite_sup - limite_inf);
            end
                      
        elseif i>j
            %para formar matrizes sim�tricas
            pdr(i,j)=pdr(j,i); 
            dist(i,j)=dist(j,i);
            prx_friis(i,j)=prx_friis(j,i);
            prx_rand(i,j)=prx_rand(j,i);
            prx(i,j)=prx(j,i);
        end
    
        etx(i,j)=1/pdr(i,j);
        
    end
end

%% Roteamento PRL com Load Balancing (Protocol for Routing of Low-Power Lossy Networks)

%pre-aloca��es
%Inicializa��o de todos os ranks de APs como zero
rank=zeros(n,1);
%representa todas as cadeias de vizinhos Uplink para Convergecast
hop_chain=zeros(n,n);
%representa a contagem de quantidades de saltos ate chegar no AP
hop_count=zeros(n,1);
%armazena o ID dos dispositivos vizinhos para redundancia de rota
vizinhos_red_rota = zeros(n, red_rota);
%armazena o rank dos dispositivos vizinhos para redundancia de rotai
ranks_red_rota = zeros(n, red_rota);
ranks_red_rota = ranks_red_rota + rank_sup;


%coleta dos vizinhos com maior PDR (menor ETX)
etx_vizinho_otimo  = min(etx);

%Inicializa��o de todos os ranks de dispositivos de campo
for i=(nap+1):n
    
    %se o disp n�o possui vizinho melhor que etx_thresh
    if etx_vizinho_otimo(i) > etx_thresh(i)
        
        %garantir que todo dispositivo tenha ao menos um vizinho
        etx_vizinho_otimo(i) = etx_thresh(i);
        
    end
    
    %rank
    rank(i)=rank_sup;
    
end

%algoritmo de RPL com Load Balancing
for rpl_count=1:rpl_iter
       
    %printar para teste somente
    %rpl_count
    
    %varrendo cada dispositivo de campo
    for i=(nap+1):n
        
        %variaveis auxiliares
        potencial_novo_rank = Inf;
        potencial_novo_vizinho = Inf;
        
        %checa se � a primeira itera��o para permitir links com APs
        if rpl_count == 1
            
            %varrendo cada vizinho
            for j=1:n

                %se o vizinho � linkavel e possui rank + ETX melhor ate agora
                if etx(i,j) < etx_thresh(i) && potencial_novo_rank > rank(j) + etx(i,j)

                    
                    %atualiza potencial novo link
                    potencial_novo_rank = rank(j) + etx(i,j);
                    potencial_novo_vizinho = j;

                end

            end
            
        %failsafe caso o dispositivo possua liga��o direta com AP
        elseif hop_count(i)==1
            
            %desfaz altera��es de variaveis auxiliares
            potencial_novo_rank = rank(i);
            potencial_novo_vizinho = hop_chain(i,1);
            %isso inibe que dispositivos que tenham se ligado a um AP
            %desistam dele depois
            
        %se nao for o primeiro ciclo e o dispositivo nao est� a um salto de
        %dist�ncia de um AP, avalia-se somente os dispositivos de campo
        %como potenciais vizinhos
        else
            %varrendo cada vizinho
            for j=(nap+1):n

                %se o vizinho � linkavel e possui rank + ETX melhor ate agora
                if etx(i,j) < etx_thresh(i) && potencial_novo_rank > rank(j) + etx(i,j)

                    %anti-deadlock
                    deadlock = false;
                    for anti_d=1:n
                        
                        %se o vizinho encontrado "j" ja possui o
                        %dispositivo "i" em seu hop_chain, "j" nao � um
                        %vizinho valido. Caso contr�rio, o pacote "deu meia
                        %volta"
                        if hop_chain(j,anti_d)==i
                            deadlock = true;
                        end
                    end
                    
                    if deadlock==false
                        %atualiza potencial novo link
                        potencial_novo_rank = rank(j) + etx(i,j);
                        potencial_novo_vizinho = j;
                    end

                end

            end
            
        end
        
        %failsafe caso nao encontre dispositivos comunicaveis
        if potencial_novo_rank > rank_sup*99
            
            %d�-se mais folga para o dispositivo
            etx_thresh(i) = etx_thresh(i) +0.25;
        
        else
            %Se chegou aqui, pode atualizar seu rank
            rank(i)=potencial_novo_rank;

            %como achou novo vizinho, precisa atualizar sua cadeia
            %se o vizinho novo � um AP:
            if potencial_novo_vizinho <= nap && potencial_novo_vizinho >0

                %primeiro, zera a cadeia original, se houvesse alguma
                hop_chain(i,:) = zeros(n,1);

                %dispositivo inclui ao AP na primeira posi�ao da cadeia
                hop_chain(i,1) = potencial_novo_vizinho;
                
                %dispositivo se inclui na segunda posi�ao da cadeia
                hop_chain(i,2) = i;

                %distancia de saltos ate o AP se torna 1
                hop_count(i) = 1;

            %se o vizinho novo nao � um AP:
            elseif potencial_novo_vizinho > nap
                %sua nova cadeia torna-se uma copia da cadeia do vizinho
                hop_chain(i,:) = hop_chain(potencial_novo_vizinho,:);
                
                %garante que o vizinho est� em sua cadeia, caso ela esteja em branco
                %(failsafe para primeiras itera�oes)
                hop_chain(i,hop_count(potencial_novo_vizinho)+1) = potencial_novo_vizinho;
                
                %dispositivo se inclui na posi�ao da cadeia uma al�m do
                %vizinho (duas a mais da distancia de saltos)
                hop_chain(i,hop_count(potencial_novo_vizinho)+2) = i;

                %atualiza distancia de saltos ate o AP
                hop_count(i) = hop_count(potencial_novo_vizinho) + 1;
            end

            %se o dispositivo achou novo vizinho e mudou sua cadeia,
            %deve-se revisar tamb�m todas as cadeias de dispositivos que o
            %t�m como vizinho

            %variavel auxiliar
            hop_chain_old = zeros(n,1);

            %varre toda a matriz hop_chain
            for linha=1:n
                for coluna=1:n

                    %checa se encontrou o dispositivo rec�m-atualizado "i" 
                    %na cadeia do dispositivo "linha", e garante que n�o � 
                    %a pr�pria cadeia de "i" rec�m-atualizada
                    if hop_chain(linha, coluna) == i && linha ~= i

                        %salva a cadeia de hops original do dispositivo "linha"
                        hop_chain_old = hop_chain(linha,:);

                        %sobrescreve a cadeia de hops do disp "linha"
                        hop_chain(linha,:) = hop_chain(i,:);

                        %variaveis auxiliares (inclusive a vari�vel "coluna" do la�o, 
                        %que n�o precisamos mais varrer pois j� encontramos o dispositivo "i")
                        coluna = coluna + 1;  %coluna da hop_chain antiga
                        aux = hop_count(i)+2; %coluna da hop_chain nova

                        %enquanto ainda h� valores na hop_chain original do
                        %dispositivo "linha"
                        while coluna<n && aux <n && hop_chain_old(coluna)~=0

                            %completa o resto da cadeia do disp "linha" com os valores
                            %da cadeia original
                            hop_chain(linha,aux)=hop_chain_old(coluna);

                            %atualiza os ranks como o rank do anterior + o ETX
                            %entre eles
                            disp_atual = hop_chain(linha,aux);
                            disp_anterior = hop_chain(linha,aux-1);
                            rank(disp_atual)= rank(disp_anterior) + etx(disp_atual,disp_anterior);

                            %incremento nas variaveis auxiliares
                            aux = aux + 1;
                            coluna = coluna+1;
                        end

                        %atualiza hop_count da hop_chain atualizada
                        hop_count(linha)=aux-2;

                    end
                end
            end
            
            if load_b>0
                %atualizar cada cadeia com o balanceamento de carga dos APs

                %para cada ponto de acesso
                for var_aux=1:nap

                    %seu rank vira o fator de Load Balancing vezes a quantidade de
                    %dispositivos linkados a ele
                    rank(var_aux)= sum(hop_chain(:,1)==var_aux)*load_b/(ndisp/nap);
                end

                %varre cada cadeia
                for var_aux=(nap+1):n

                    %variavel auxiliar
                    aux=2;

                    %enquanto nao chegou ao fim da cadeia
                    while hop_chain(var_aux,aux)~=0 && aux<n

                        %atualiza o rank de cada dispositivo de campo a
                        %partir do rank rec�m-atualizado de seu AP
                        rank(hop_chain(var_aux,aux)) = rank(hop_chain(var_aux,aux-1))+etx(hop_chain(var_aux,aux),hop_chain(var_aux,aux-1));

                        %atualiza variavel auxiliar
                        aux = aux+1;
                    end
                end
            end
        end 
    end
    
end

%para estabelecer as rotas redundantes
if red_rota > 0

        
    %varrendo os dispositivos
    for i=(nap+1):n
           
        %dispositivos com salto 1 nao precisam de redundancia
        if hop_count(i)>1
               
                %varrendo os vizinhos
                for j=(nap+1):n
                   
                    %se o vizinho � linkavel e possui rank + ETX melhor ate agora
                    if etx(i,j) < etx_thresh(i) && ranks_red_rota(i,red_rota) > rank(j) + etx(i,j)

                        %anti-deadlock
                        deadlock = false;
                        
                        %se o vizinho encontrado ja � o proximo
                        %na hop_chain do dispositivo, nao faz sentido
                        %consider�-lo como rota redundante
                        if hop_chain(i,hop_count(i))==j
                            deadlock = true;
                        else
                            for anti_d=1:n

                                %se o vizinho encontrado "j" ja possui o
                                %dispositivo "i" em seu hop_chain, "j" nao � um
                                %vizinho valido. Caso contr�rio, o pacote "deu meia
                                %volta"
                                if hop_chain(j,anti_d)==i
                                    deadlock = true;
                                end
                            
                            end
                        end
                        
                        if deadlock==false
                           
                            %varre todos os ranks de redundancia atuais
                            %para encontrar o qu�o melhor � esse novo
                            %vizinho com rela��o aos vizinhos redundantes
                            %atuais 
                            aux=red_rota+1; 
                            for k=1:red_rota
                                 
                                if rank(j) + etx(i,j) < ranks_red_rota(i,red_rota +1 -k)
                                    
                                    aux = aux-1;
                                    
                                end
                                 
                            end
                            
                                
                            if red_rota - aux > 0

                                for k=1:(red_rota-aux)

                                    ranks_red_rota(i,red_rota+1-k) = ranks_red_rota(i,red_rota-k);
                                    vizinhos_red_rota(i,red_rota+1-k) = vizinhos_red_rota(i,red_rota-k);

                                end

                            end

                            ranks_red_rota(i,aux) = rank(j) + etx(i,j);
                            vizinhos_red_rota(i,aux) = j;
                            
                            
                            
                            
                            
                        end
                    end
                
                end
            
            
        
        end
        
    end
    
end

etx_vizinho_final = zeros(n,3);

for i=(nap+1):n
    
    etx_vizinho_final(i,1)=i;
    etx_vizinho_final(i,2)=hop_chain(i,hop_count(i));
    etx_vizinho_final(i,3)=etx(i,etx_vizinho_final(i,2));
end

%% Plot dos Dispositivos

%plot dos nodos
plot (pos(:,1),pos(:,2),'.');           
hold on

%plot dos pontos de acesso
for i=1:nap
%     plot (pos(i,1),pos(i,2),'ro');      
end

%plot das arestas
for i=(nap+1):n
    if hop_count(i)==1
        color='r';
        LineWidth = 1.5;

    else
        color=':k';
        LineWidth = 0.05;
    end

    plot([pos(i,1) pos(hop_chain(i,hop_count(i)),1)],[pos(i,2) pos(hop_chain(i,hop_count(i)),2)],color,"LineWidth",LineWidth);
    
    if red_rota>0 && hop_count(i)~=1
        
        plot([pos(i,1) pos(vizinhos_red_rota(i,1),1)],[pos(i,2) pos(vizinhos_red_rota(i,1),2)],'--');
    
    end
end
hold off
%% Displays para testes

carga_ap=zeros(nap,1);
for i=1:nap
    carga_ap(i) = sum(hop_chain(:,1)==i);
    disp(['Carga dos AP ', num2str(i),' = ', num2str(carga_ap(i))])
end

for i=1:nap
    disp(['Posi��o do AP ',num2str(i),' = (',num2str(pos(i,1)),', ',num2str(pos(i,2)),')'])
end

max_hop_chain_first = max(hop_chain(:,1));
disp(['Maior dispositivo na cabeceira da cadeia = ',num2str(max_hop_chain_first)])

max_hop_count = max(hop_count);
disp(['Maior salto de dispositivos = ', num2str(max_hop_count)])

max_rank = max(rank);
%disp(['Maior rank = ', num2str(max_rank)])

%% Escalonamento (apenas para Uplink/Convergecast)

%quantos pacotes ser�o gerados na simula��o por dispositivo
n_pacotes = ndisp*n_ciclos;

%representar a instancia de cada pacote
%suas colunas representam: "origem", "destino", "ciclo de nascimento", 
%"dispositivo atual", "entrega(1), caducou TTL(2), buffer cheio (3)"
pacotes = zeros(n_pacotes,5);

%armazena a qtde corrente de pacotes ja criados
pacotes_criados = 0;

%representar o buffer dos dispositivos
buffers = zeros(n,tam_buffer);

%representar o quao cheio est� cada buffer
buffer_count = zeros(n,1);

%contador de falhas de transmissao. se em toda a redundancia de links "red_link"
%nenhum pacote foi enviado, o dispositivo busca uma rota redundante
count_transm_fail = zeros(n,1);

%bit que representa qual destinat�rio ponto a ponto cada dipositivo vai
%escolher. "0" representa a rota principal e cada numero subsequente � o
%�ndice de rota redundante a ser utilizada (1a rota, 2a rota, ...)
destino_red = zeros(n,1);

%para provisionar a qtde minima de slots em um ciclo 
%que atenda ao numero de rotas que passam por cada dispositivo, 
%considerando redundancia de links 
prov_link=zeros(n,1);
for i=(nap+1):n
    
    %quantas rotas passam por esse dispositivo
    prov_link(i) = sum(hop_chain(:,hop_count(i)+1)==i);
    
   
end

%esse array armazena a sequencia de slots a serem processados pelo
%simulador. pode representar um ou mais superframes 
superframe = zeros(sum(prov_link),1);
prov_link_aux = prov_link;
i=1;
while i<=sum(prov_link)
    
    %varrendo dispositivos
    for j=(nap+1):n
        
        %se ainda precisa provisionar slots para o dispositivo "j"
        if prov_link_aux(j)>0
            
            %o slot "i" � provisionado para o dispositivo "j"
            superframe(i)=j;
            i = i + 1;
            
        end
        
    end
    
    prov_link_aux = prov_link_aux - 1;
end

%la�o de opera��o da rede
for sched_count=1:n_ciclos
    
    %print para teste
    %sched_count
    
    %cada dispositivo varre seu buffer em busca de pacotes antigos demais
    %para serem validos ainda
    i=1;
    while i<=n_pacotes && pacotes(i,1)~=0
        
        %se o pacote n�o tem mais chance de chegar ao destino antes de
        %estourar o TTL
        if ttl < sched_count - pacotes(i,3) + hop_count(pacotes(i,4)) && pacotes(i,5) ==0
            
            %pacote � indicado como falha por caducar TTL
            pacotes(i,5) = 2;
            
            %print para teste
%             disp(['Pacote ',num2str(i),' caducou no ciclo ',num2str(sched_count)]);
            
            pacote_ja_removido = false;
            
            %pacote � removido do buffer do dispositivo
            for j=1:tam_buffer
                
                %encontrar o pacote no buffer
                if buffers(pacotes(i,4),j)==i && pacote_ja_removido==false
                    
                    %se for o �ltimo pacote do buffer
                    if j==tam_buffer
                        
                        %somente apaga esse pacote do buffer
                        buffers(pacotes(i,4),j)=0;
                    
                    %se nao for o �ltimo
                    else
                        
                        %ate a penultima casa do buffer
                        for k=j:tam_buffer-1
                            
                            %adianta o pacote da posi�ao anterior
                            buffers(pacotes(i,4),k) = buffers(pacotes(i,4),k+1);
                            
                        end
                        
                        %zera a ultima posi��o, s� pra garantir caso o
                        %buffer estivesse cheio
                        buffers(pacotes(i,4),tam_buffer) = 0;
                        
                        %for�a o fim do loop
                        pacote_ja_removido=true;
                        
                        
                    end
                    
                end
                
            end
             
        end
        
        i=i+1;
        
    end
    
    %cada dispositivo de campo cria um pacote novo que precisa ser
    %encaminhado ao seu AP
    for disp_criando_pacote=(nap+1):n
        
        %aumenta o contador de pacotes criados
        pacotes_criados = pacotes_criados + 1;

        %seta origem do pacote
        pacotes(pacotes_criados,1) = disp_criando_pacote;

        %seta destino do pacote como o AP, na cabeceira da cadeia
        pacotes(pacotes_criados,2) = hop_chain(disp_criando_pacote,1);

        %seta a data de nascimento do pacote
        pacotes(pacotes_criados,3) = sched_count;
        
        %seta ao dispositivo atual no qual o pacote se encontra (o de
        %origem)
        pacotes(pacotes_criados,4) = disp_criando_pacote;
        
        %se o buffer do dispositivo possui loca��o disponivel
        if buffers(disp_criando_pacote, tam_buffer)==0
          
            achou_posicao = false;
            %salva o pacote no buffer do dispositivo que o criou
            for varre_buffer=1:tam_buffer
                
                %se achou a primeira posi��o vazia no buffer
                if buffers(disp_criando_pacote,varre_buffer)==0 && achou_posicao==false
                    
                    %salva o indice do pacote recem-criado
                    buffers(disp_criando_pacote,varre_buffer)=pacotes_criados;
                    
                    %for�a o fim do loop
                    achou_posicao = true;
                    
                end
            end
            
        %mas se o buffer do dispositivo ja estava cheio    
        else
            
            %automaticamente, o pacote � descartado e n�o � salvo no buffer
            pacotes(pacotes_criados,5)=3;
            
%             disp(['Pacote ',num2str(pacotes_criados),' foi abortado em cria��o por buffer cheio no ciclo ',num2str(sched_count)]);
            
        end
        
    end
    
    %la�o que varre os slots do(s) superframe(s)
    for superframe_count=1:sum(prov_link)
    
        %dispositivo para o qual este slot est� provisionado
        disp_emissor=superframe(superframe_count);
        
        %teste
%         if disp_emissor == 357
%             eh_agora=true;
%         end
        
                    %para cada redundancia de link arbitrada
                    for red_link_count=1:red_link
                        
                            %checa se ha algum pacote no buffer que ainda precisa ser
                            %enviado
                            if buffers(disp_emissor,1)~=0

                                    %numero aleatorio a ser comparado com o PDR da dupla de
                                    %dispositivos sendo verificada (quanto maior, pior)
                                    chance_sucesso = (randi(100,1,1)-fator_dif_transm)/100;

                                    %salva o indice do pacote a ser enviado, que est� na primeira
                                    %posi�ao de seu buffer (seguindo FIFO)
                                    pacote_enviado = buffers(disp_emissor,1);


                                    %salva qual o endere�o do proximo destinatario ponto-a-ponto
                                    %se o dispositivo emissor est� programado
                                    %para mandar na rota principal
                                    if destino_red(disp_emissor) == 0

                                        %o dispositivo receptor � salvo como o
                                        %proximo na hop_chain
                                        disp_receptor = hop_chain(disp_emissor,hop_count(disp_emissor));

                                    %se o dispositivo emissor est� programado para mandar para alguma rota redundante    
                                    else

                                        disp_receptor = vizinhos_red_rota(disp_emissor,destino_red(disp_emissor));

                                    end

                                    %se o buffer do receptor nao est� cheio
                                    if buffers(disp_receptor, tam_buffer)==0

                                        %se a comunica��o foi bem sucedida
                                        if chance_sucesso < pdr(disp_emissor,disp_receptor)

                                            count_transm_fail(disp_emissor) = 0;

                                            %se o dispositivo receptor � um AP
                                            %(destino fim-a-fim)
                                            if disp_receptor<=nap

                                                %pacote � considerado como sucesso
                                                pacotes(pacote_enviado,5)=1;

                                                %disp(['Pacote ',num2str(pacote_enviado),' chegou ao AP no ciclo ',num2str(sched_count),'!']);

                                            %se for s� um intermediario no roteamento, precisa salvar
                                            %no buffer
                                            else

                                                achou_pacote=false;

                                                %salva o pacote no buffer do dispositivo que o recebeu
                                                for varre_buffer=1:tam_buffer

                                                    %se achou a primeira posi��o vazia no buffer
                                                    if buffers(disp_receptor,varre_buffer)==0 && achou_pacote==false

                                                        %salva o indice do pacote recebido
                                                        buffers(disp_receptor,varre_buffer)=pacote_enviado;

                                                        %for�a o fim do loop
                                                        achou_pacote = true;

                                                    end
                                                end

                                                %disp(['Pacote ',num2str(pacote_enviado),' enviado com sucesso pro vizinho ponto-a-ponto no ciclo ',num2str(sched_count)]);

                                            end

                                            %apaga o pacote do dispositivo emissor
                                            %ate a penultima casa do buffer
                                            for k=1:tam_buffer-1

                                                %adianta o pacote da posi�ao anterior
                                                buffers(disp_emissor,k) = buffers(disp_emissor,k+1);

                                            end

                                            %zera a ultima posi��o, s� pra garantir caso o
                                            %buffer estivesse cheio
                                            buffers(disp_emissor,tam_buffer) = 0;

                                            %atualiza o pacote para indicar em qual dispositivo ele se
                                            %encontra
                                            pacotes(pacote_enviado,4)=disp_receptor;




                                        %se a comunica��o falhou
                                        else
                                            
                                            count_transm_fail(disp_emissor) = count_transm_fail(disp_emissor)+1;
                                            %disp(['Pacote ',num2str(pacote_enviado),' falhou por PDR no ciclo ',num2str(sched_count)])

                                        end



                                    %se o buffer do receptor ja estava cheio                
                                    else
                                        
                                        count_transm_fail(disp_emissor) = count_transm_fail(disp_emissor)+1;
                                        %disp(['Pacote ',num2str(pacote_enviado),' falhou por buffer cheio no vizinho no ciclo ',num2str(sched_count)])

                                    end
                                    
                                    %se mesmo com redundancia de links o pacote nao foi
                                    %transmitido
                                    if count_transm_fail(disp_emissor) >=red_link

                                            count_transm_fail(disp_emissor) = 0;

                                            %redundancia de rotas s� vale para dispositivos nao
                                            %conectados a um AP
                                            if hop_count(disp_emissor)>1 

                                                %se o dispositivo ja passou por todas as rotas
                                                %redundantes disponiveis, volta para a rota original
                                                if destino_red(disp_emissor)+1>red_rota || vizinhos_red_rota(disp_emissor, destino_red(disp_emissor)+1)==0

                                                    destino_red(disp_emissor)=0;
                                                    %disp(['O dispositivo ',num2str(disp_emissor),' voltou para sua rota original'])

                                                else

                                                    %a nova transmiss�o do dispositivo ser� para a proxima rota redundante
                                                    destino_red(disp_emissor)=destino_red(disp_emissor)+1;
                                                    %disp(['O dispositivo ',num2str(disp_emissor),' foi para a rota redundante ',num2str(destino_red(disp_emissor))])


                                                end

                                            end
                                            
                                    end
                                    
                                    

                            end
                    

                    end
                    
                    
        
    end
   
end

%% Centralidades

%inicializa��o
cent_grau = zeros(n,1);
cent_prox = zeros(n,1);
cent_int = zeros(n,1);
cent_aut = zeros(n,1);

%para centralidade de proximidade
    %varrendo dispositivos
    for i=(nap+1):n

        %varrendo os vizinhos
        for j=(1):n

            %se dispositivo e vizinhos est�o no mesmo AP (e n�o s�o o mesmo
            %dispositivo)
            if hop_chain(i,1)==hop_chain(j,1) && i~=j

                %variavel auxiliar para varrer as hop chains. k=1 n�o �
                %necess�rio pois ja sabemos que estao no mesmo AP
                k=1;

                %vari�vel auxiliar para for�ar fim do la�o
                end_loop=false;

                %se varrermos toda a hop chain do dispositivo i e for toda
                %igual � hop chain de j, sabemos que i � um predecessor de j
                while k<=hop_count(i)+1 && end_loop==false

                    %o momento em que as hop chains se diferenciam � o momento
                    %de bifurca��o nas rotas de uplink de i e j
                    if hop_chain(i,k) ~= hop_chain(j,k)

                        %se hop chain de j zerou, j � predecessor de i
                        if hop_chain(j,k)==0

                            %somar o ETX do caminho direto de j descendo ate i
                            while hop_chain(i,k)~=0

                               cent_prox(i)=cent_prox(i)+etx(hop_chain(i,k-1),hop_chain(i,k));
                               k=k+1; 
                            end

                        %caso contr�rio, � porque h� uma bifurca��o entre eles
                        else
                            aux=k;
                            %somar o caminho do ponto de bifurca��o at� j
                            while hop_chain(j,k)~=0

                               cent_prox(i)=cent_prox(i)+etx(hop_chain(j,k-1),hop_chain(j,k));
                               k=k+1; 
                            end

                            %somar o caminho do ponto de bifurca��o at� i
                            while hop_chain(i,aux)~=0

                               cent_prox(i)=cent_prox(i)+etx(hop_chain(i,aux-1),hop_chain(i,aux));
                               aux=aux+1; 
                            end


                        end

                        end_loop=true;
                    end

                    k=k+1;
                end

                %se end_loop ainda � false, sabemos que i � predecessor de j
                if end_loop == false

                    %somar o ETX do caminho direto de i descendo ate j
                    while hop_chain(j,k)~=0

                       cent_prox(i)=cent_prox(i)+etx(hop_chain(j,k-1),hop_chain(j,k));
                       k=k+1; 
                    end

                end


            end

        end

    end

%varrendo cadeias
for i=(nap+1):n
    
    %para centralidade de grau
        cent_grau(hop_chain(i,hop_count(i)))=1+cent_grau(hop_chain(i,hop_count(i)));
    
    %para centralidade de intermedia��o
        %vari�vel auxiliar
        j=1;

        %enquanto nao chegou no fim da cadeia
        while hop_chain(i,j)>0

            %aumenta a centralidade de intermedia��o do dispositivo identificado na
            %cadeia
            cent_int(hop_chain(i,j))=cent_int(hop_chain(i,j))+1;
            j=j+1;
        end

end

%completando o c�lculo, dividindo todas as centralidades de intermedia��o
%pelo numero de rotas
% cent_grau = cent_int/ndisp;

% for i=1:nap
%     cent_int(i)=cent_int(i)/carga_ap(i);
% end

for i=(nap+1):n
    cent_int(i)=cent_int(i)/carga_ap(hop_chain(i,1));
    cent_prox(i)=cent_prox(i)/carga_ap(hop_chain(i,1));
end

% cent_grau=cent_grau*100;
cent_grau=cent_grau*100/n;
cent_int=cent_int*100;
% cent_prox=cent_prox*100;

%as centralidades dos pontos de acesso nao sao relevantes
for i=1:nap
    cent_grau(i)=0;
    cent_int(i)=0;
end


%% Displays para testes

[max_prov, max_prov_i] = max(prov_link);
disp(['Maior provisionamento = ',num2str(max_prov),' (dispositivo ',num2str(max_prov_i),')']);


pacotes_em_transito = sum(pacotes(:,5)==0);
pacotes_bem_sucedidos = sum(pacotes(:,5)==1);
pacotes_perdidos_ttl = sum(pacotes(:,5)==2);
pacotes_perdidos_buffer = sum(pacotes(:,5)==3);
pactes_destinados = pacotes_bem_sucedidos+pacotes_perdidos_ttl+pacotes_perdidos_buffer;
confiabilidade = 100*pacotes_bem_sucedidos/pactes_destinados;

disp(['Pacotes em transito = ',num2str(100*pacotes_em_transito/pacotes_criados),' %']);
disp(['Pacotes perdidos por TTL = ',num2str(100*pacotes_perdidos_ttl/pactes_destinados),' %']);
disp(['Pacotes perdidos por buffer cheio = ',num2str(100*pacotes_perdidos_buffer/pactes_destinados),' %']);

disp(['Confiabilidade = ',num2str(confiabilidade),' %']);

pdr_roteamento = zeros(ndisp,1);
for i=(nap+1):n
    pdr_roteamento(i-nap)=pdr(i,hop_chain(i,hop_count(i)));
end

disp(['M�dia de PDR = ',num2str(mean(100*pdr_roteamento)),' %']);

%% Display An�lise Centralidades

%para normalizar as centralidades
[max_grau,max_grau_i]=max(cent_grau);
[max_int,max_int_i]=max(cent_int);
[max_prox,max_prox_i]=max(cent_prox);
[max_rank,max_rank_i]=max(rank);

%variavel de indice para gerar tabela de dispositivos que geraram papotes
%perdidos
disp_geraram_falha=1;

%tabela de dispositivos que geraram papotes
%perdidos (grau, int, prox, rank)
analise_centralidades = zeros(ndisp,4);

%varrer todos os pacotes
for i=1:pacotes_criados
        
    %se o pacote foi perdido
    if pacotes(i,5)==2 || pacotes(i,5)==3
        
        %recolhe qual dispositivo foi o seu gerador
        disp_origem=pacotes(i,1);
        
        %recolhe qual dispositivo est� na ponta da cadeia
        disp_ponta=hop_chain(disp_origem,2);
        
%         %recolhe qual dispositivo possui a maior CG/CI, ignorando o NAP
%         maior_cg_nodo_i = 2;
%         maior_ci_nodo_i = 2;
% 
%         if hop_count(disp_origem)>1
%             for j=3:hop_count(disp_origem+1)
%                    if cent_grau(hop_chain(disp_origem,maior_cg_nodo_i)) < cent_grau(hop_chain(disp_origem,j))
%                        maior_cg_nodo_i = j;
%                    end
%                    
%                    if cent_int(hop_chain(disp_origem,maior_ci_nodo_i)) < cent_int(hop_chain(disp_origem,j))
%                        maior_ci_nodo_i = j;
%                    end
%                    
%                    
%             end
%         end
        
        %salva a centralidade de grau e de intermedia�ao do dispositivo da
        %ponta
        analise_centralidades(disp_geraram_falha,1)=cent_grau(disp_ponta)/max_grau;
        analise_centralidades(disp_geraram_falha,2)=cent_int(disp_ponta)/max_int;
        
        %salva a centralidade de proximidade e rank do dispositivo de origem do
        %pacote falho
        analise_centralidades(disp_geraram_falha,3)=cent_prox(disp_origem)/max_prox;
        analise_centralidades(disp_geraram_falha,4)=rank(disp_origem)/max_rank;
        
        disp_geraram_falha=disp_geraram_falha+1;
        
    end
end



