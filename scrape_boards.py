import requests
from bs4 import BeautifulSoup

# URL of the MicroPython downloads page
url = "https://micropython.org/download/"

# Send a GET request to the page
response = requests.get(url)

def write_to_markdown_file(boards, filename):
    with open(filename, 'w') as file:
        # Write the markdown table header
        file.write("| Product Name | Vendor|\n")
        file.write("|--------------|--------|\n")

        # Write each row of the table
        for product, vendor, link in boards:
            file.write(f"| {product} | [{vendor}](https://micropython.org/download/{link})|\n")
           
        file.write('{:class="table table-striped"}\n')
# Check if the request was successful
if response.status_code == 200:
    # Parse the HTML content of the page
    soup = BeautifulSoup(response.text, 'html.parser')

    # Find all elements with the class 'board-card'
    board_cards = soup.find_all(class_='board-card')

    # Create a list to hold the board details
    boards = []

    # Loop through each board card and extract the information
    for card in board_cards:
        # Extract the board image URL
        image_tag = card.find(class_='board-image').find('img')

        # Extract the board product name
        board_product = card.find(class_='board-product').text.strip()

        # Extract the board vendor name
        board_vendor = card.find(class_='board-vendor').text.strip()

        # Extract the link to the board
        board_link = card['href']

        # Add the information to the boards list
        boards.append((board_product, board_vendor, board_link))

    # Print the markdown table header
    print("| Product Name | Vendor |")
    print("|--------------|--------|")

    # Print each row of the table
    for product, vendor, link in boards:
        print(f"| {product} | [{vendor}](https://micropython.org/download/{link})|")
    
    # Write the boards list to a markdown file
    write_to_markdown_file(boards, filename="web/_includes/micropython_boards.md")
else:
    print("Failed to retrieve the data from the website.")
