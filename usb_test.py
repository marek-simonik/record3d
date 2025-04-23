import usb.core
import usb.util

# Trova tutte le periferiche USB connesse
devices = usb.core.find(find_all=True)

# Itera sulle periferiche e stampa le informazioni
for dev in devices:
  try:
    # Ottieni il nome del produttore e del prodotto
    manufacturer = usb.util.get_string(dev, dev.iManufacturer)
    product = usb.util.get_string(dev, dev.iProduct)

    # Stampa le informazioni sul dispositivo
    print("Dispositivo trovato:")
    print(f"  ID Vendor: {hex(dev.idVendor)}")
    print(f"  ID Prodotto: {hex(dev.idProduct)}")
    if manufacturer:
      print(f"  Produttore: {manufacturer}")
    if product:
      print(f"  Prodotto: {product}")
    print("-" * 20)

  except Exception as e:
    print(f"Errore durante la lettura delle informazioni del dispositivo: {e}")