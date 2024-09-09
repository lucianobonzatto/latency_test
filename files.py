import os
import pandas as pd

caminho_pasta = 'data\stress'
dataframes = []

for arquivo in os.listdir(caminho_pasta):
    if arquivo.endswith('.csv'):
        caminho_arquivo = os.path.join(caminho_pasta, arquivo)
        df = pd.read_csv(caminho_arquivo)  # Lê o arquivo .csv
        dataframes.append(df)  # Adiciona o DataFrame à lista

print(dataframes.__len__())

if dataframes:
    data = pd.concat(dataframes, ignore_index=True)
    data.to_csv('all_data.csv', index=False)
else:
    print("Nenhum arquivo CSV encontrado na pasta 'data'.")
