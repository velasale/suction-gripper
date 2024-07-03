

money = 19
money = 11
money = 7
# money = 5

expenses_per_month = 2

rates = [9, 9.1, 9.9, 10, 10.1, 10.05]

best_interest = 0

for investment in range(1, money):

    for month in range(1, 6):
       
        balance = (money - investment) * 1e6
        expenses = month * expenses_per_month * 1e6

        # Check conditions
        if balance > expenses:

            interests = investment * 1e6 * (rates[month-1] / 100) * 1 / 12 * month

            # keep the best
            if interests > best_interest:
                best_interest = interests
                best_month = month
                best_investment = investment
                best_balance = balance
                best_rate = rates[month-1]

print('\nBest possible CDT given an initial amount of %i million COP' % money)
print('Invest %i million COP, during %i months, with a %.2f rate. Interests: %.2f' % (best_investment, best_month, best_rate, best_interest))
print('Money available: %.2f million COP ' % (best_balance/1e6))
