# Проброс HTTP (80 → 8081, где редирект-сервер)
sudo iptables -t nat -A PREROUTING -p tcp --dport 80 -j REDIRECT --to-port 8081

# Проброс HTTPS (443 → 8080, где основной сервер)
sudo iptables -t nat -A PREROUTING -p tcp --dport 443 -j REDIRECT --to-port 8080

# Сохраните правила (чтобы не пропали после перезагрузки)
sudo apt install iptables-persistent
sudo netfilter-persistent save